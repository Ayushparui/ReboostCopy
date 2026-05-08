/**
 * @file Main.cpp
 * Implementation of the main() function for starting and stopping the module framework.
 * @author Colin Graf
 * @author Rudi Villing (modifications)
 */

#include "Framework/Robot.h"
#include "Framework/Robots.h"
#include "Framework/Settings.h"
#include "Math/Constants.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Streaming/FunctionList.h"
#include "TextLogging/TextLogging.h"

#include <booster/idl/b1/LowCmd.h>
#include <booster/robot/b1/b1_loco_client.hpp>
#include <booster/robot/channel/channel_publisher.hpp>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <sys/file.h> // flock
#include <fcntl.h>
#include <pthread.h>
#include <sys/wait.h>
#include <unistd.h>

static pid_t roboeireannPid = 0;
static Robot* robot = nullptr;
static bool run = true;
static pthread_t mainThread;
static bool shutdownBooster = false;


DECL_TLOGGER(tlogger, "Main", TextLogging::INFO);


SystemCall::Mode SystemCall::getMode()
{
  return physicalRobot;
}

static void roboeireannStart(const Settings& settings)
{
  TLOGI(tlogger, "Roboeireann: Start.");

  robot = new Robot(settings, std::string());
  robot->start();
}

static void roboeireannStop()
{
  TLOGI(tlogger, "Roboeireann: Stop.");

  robot->announceStop();
  robot->stop();
  delete robot;
  robot = nullptr;
}

static void sighandlerShutdown(int sig)
{
  if(pthread_self() != mainThread)
  {
    TLOGI(tlogger, "Caught termination/interrupt signal ({}) on non-main thread -- redirecting to main thread...",
          Assert::getSignalName(sig));

    shutdownBooster = true;
    pthread_kill(mainThread, sig);
  }
  else
  {
    if(run)
    {
      TLOGI(tlogger, "Caught (normal) termination/interrupt signal ({}) -- Shutting down executable...",
            Assert::getSignalName(sig));
      TextLogging::requestAsyncOutput(false); // output any async log messages
      fflush(NULL);                           // flush all streams
    }
    run = false;
  }
}

static void sighandlerRedirect(int)
{
  run = false;
}

static Settings::RobotType getRobotType(const std::string& robotName)
{
  const std::string bhdir = File::getBHDir();
  InMapFile robotsStream(bhdir + "/Config/Robots/robots.cfg");
  if (!robotsStream.exists())
    TLOGW(tlogger, "Could not load robots.cfg");
  else
  {
    Robots robots;
    robotsStream >> robots;
    for(const Robots::RobotId& robot : robots.robotsIds)
      if(robot.name == robotName)
        return robot.robotType;
    TLOGW(tlogger, "Could not find robot name {} in robots.cfg!", robotName);
  }
  return Settings::nao;
}

int main(int argc, char* argv[])
{
  booster::robot::b1::B1LocoClient client;
  {
    // Set stdout to be unbuffered.
    // This has previously been done using stdbuf, but this does not work for a 64-bit program on a 32-bit system.
    setvbuf(stdout, nullptr, _IONBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);

    mainThread = pthread_self();
    Thread::nameCurrentThread("Main"); // for text logging output

    // parse command-line arguments
    bool watchdog = false;
    const char* bhDir = "/home/booster";

    for(int i = 1; i < argc; ++i)
      if(!strcmp(argv[i], "-c") && i + 1 < argc)
        bhDir = argv[++i];
      else if(!strcmp(argv[i], "-w"))
        watchdog = true;
      else
      {
        TLOGI(tlogger, "\n\n\
Usage: {} [-c <dir>] [-w]\n\
    -c <dir>      specify a base config directory (default is /home/booster)\n\
    -w            use a watchdog for crash recovery and creating trace dumps\n", argv[0]);
        exit(EXIT_FAILURE);
      }

    // avoid duplicated instances
    int fd = open("/tmp/roboeireann_lock", O_CREAT, 0600);
    if(fd == -1 || flock(fd, LOCK_EX | LOCK_NB) == -1)
    {
      TLOGE(tlogger, "There is already an instance of this process running!");
      exit(EXIT_FAILURE);
    }

    // change working directory
    if(*bhDir && chdir(bhDir) != 0)
    {
      TLOGE(tlogger, "chdir to base config directory failed!\n");
      exit(EXIT_FAILURE);
    }

    // the watchdog
    if(watchdog)
    {
      for(;;)
      {
        roboeireannPid = fork();
        if(roboeireannPid == -1)
          exit(EXIT_FAILURE);
        else if(roboeireannPid == 0) // this is the newly forked child
          break;

        // this is the parent code continuing
        Thread::nameCurrentThread("WatchdogMain");
        TLOGI(tlogger, "Starting watchdog process");

        int status;
        signal(SIGTERM, sighandlerRedirect);
        signal(SIGINT, sighandlerRedirect);
        if(waitpid(roboeireannPid, &status, 0) != roboeireannPid)
        {
          exit(EXIT_FAILURE);
        }
        signal(SIGTERM, SIG_DFL);
        signal(SIGINT, SIG_DFL);

        // we only get here when the child process exited
        // was it requested/normal exit?
        bool normalExit = !run || (WIFEXITED(status) && WEXITSTATUS(status) == EXIT_SUCCESS);

        TLOGI(tlogger, "Child process has exited, requested={}, normalExit={}, exitStatus={}", !run,
              WIFEXITED(status) == 0, WEXITSTATUS(status));

        // dump trace and assert trace
        if(!normalExit)
        {
          // Dump to file first, because writing to stderr may fail for various reasons.
          Assert::logDump(false, WIFSIGNALED(status) ? int(WTERMSIG(status)) : 0);
          Assert::logDump(true, WIFSIGNALED(status) ? int(WTERMSIG(status)) : 0);

          // Switch Booster back to damping mode.
          booster::robot::ChannelFactory::Instance()->Init(0);
          booster::robot::b1::B1LocoClient client;
          client.Init();
          usleep(10000);
          TLOGI(tlogger, "Switching robot back to damping mode");
          const int result = client.ChangeMode(booster::robot::RobotMode::kDamping);
          if(result)
          {
            TLOGE(tlogger, "Switching robot back to damping mode failed with error {}!", result);
            return EXIT_FAILURE;
          }
        }

        // quit
        exit(WIFEXITED(status) && normalExit ? WEXITSTATUS(status) : EXIT_FAILURE);
      }
    }

    // =======================================================================
    // The child process code
    // =======================================================================

    // Initialize connection to robot at realtime priority.
    sched_param param;
    param.sched_priority = 21; // One more than Motion
    if(pthread_setschedparam(pthread_self(), SCHED_FIFO, &param))
    {
      TLOGE(tlogger, "Raising priority failed!");
      return EXIT_FAILURE;
    }
    booster::robot::ChannelFactory::Instance()->Init(0);
    param.sched_priority = 0;
    if(pthread_setschedparam(pthread_self(), SCHED_OTHER, &param))
    {
      TLOGE(tlogger, "Lowering priority failed!");
      return EXIT_FAILURE;
    }

    client.Init();
    usleep(100000);

    // Prepare motor commands with zero p and d values, i.e. motors off.
    booster_interface::msg::LowCmd lowCmd;
    lowCmd.cmd_type(booster_interface::msg::SERIAL);
    lowCmd.motor_cmd(std::vector<booster_interface::msg::MotorCmd>(booster::robot::b1::kJointCnt));

    // Deactivate all motors before switching to custom mode, because robot would otherwise apply previous commands.
    booster::robot::ChannelPublisher<booster_interface::msg::LowCmd> lowCmdPublisher(booster::robot::b1::kTopicJointCtrl);
    lowCmdPublisher.InitChannel();
    if(!lowCmdPublisher.Write(&lowCmd))
    {
      TLOGE(tlogger, "Resetting motor commands failed!");
      return EXIT_FAILURE;
    }

    BH_TRACE_INIT("main");

    // Acquire static data, e.g. about types
    FunctionList::execute();

    const std::string name = SystemCall::getHostName();
    Settings settings(name, name, Constants::boosterMotionCycleTime, getRobotType(name));
    if(settings.playerNumber < 0 || settings.bodyName.empty())
      return EXIT_FAILURE;


    TextLogging::logStartupTime(tlogger, TextLogging::INFO);


    // print status information
    TLOGI(tlogger, "Hi, I am {}.", settings.headName.c_str());
    TLOGI(tlogger, "teamNumber {}", settings.teamNumber);
    TLOGI(tlogger, "fieldPlayerColor {}", TypeRegistry::getEnumName(settings.fieldPlayerColor));
    TLOGI(tlogger, "goalkeeperColor {}", TypeRegistry::getEnumName(settings.goalkeeperColor));
    TLOGI(tlogger, "playerNumber {}", settings.playerNumber);
    TLOGI(tlogger, "location {}", settings.location.c_str());
    TLOGI(tlogger, "scenario {}", settings.scenario.c_str());
    TLOGI(tlogger, "magicNumber {}", settings.magicNumber);

    // put the current body name in a file (at the moment only used in downloadCalibration)
    OutTextRawFile("/var/volatile/tmp/bodyName.txt") << settings.bodyName;

    // register signal handler for ctrl+c and termination signal
    signal(SIGTERM, sighandlerShutdown);
    signal(SIGINT, sighandlerShutdown);

    roboeireannStart(settings);

    // settings go out of scope here, but everything that needs it later makes a copy.
  }

  while(run)
    pause();

  roboeireannStop();

  TextLogging::requestAsyncOutput(false); // back to sync logging in the calling thread

  const int result = client.ChangeMode(booster::robot::RobotMode::kDamping);
  if(result)
    TLOGE(tlogger, "Switching robot back to damping mode failed with error {}\n", result);

  if (shutdownBooster)
  {
    TLOGI(tlogger, "Preparing to power-off the robot");
    fflush(NULL);

    Thread::sleep(500);

    // to be sure to be sure...
    TLOGI(tlogger, "Power off now");
    fflush(NULL);

    static_cast<void>(!system("sudo systemctl poweroff &"));
  }

  return EXIT_SUCCESS;
}
