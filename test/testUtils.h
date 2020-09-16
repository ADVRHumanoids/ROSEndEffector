#ifndef TESTUTILS_H
#define TESTUTILS_H

#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/wait.h>

//for the function prepareROSForTests
#include <ros/ros.h>
#include <ros_end_effector/Utils.h>


/** Utils funcion to create process to run roscore,
 * gently copied from https://github.com/ADVRHumanoids/CartesianInterface/blob/refactor2020/tests/testutils.h
 */

namespace ROSEE {
    
namespace TestUtils {

class Process
{

public:

    Process(std::vector<std::string> args);

    int wait();

    void kill(int signal = SIGTERM);

    ~Process();

private:

    std::string _name;
    pid_t _pid;

};

Process::Process(std::vector<std::string>  args):
    _name(args[0])
{
    std::vector<const char *> args_cstr;
    for(auto& a : args) args_cstr.push_back(a.c_str());
    args_cstr.push_back(nullptr);

    char ** argv = (char**)args_cstr.data();

    _pid = ::fork();

    if(_pid == -1)
    {
        perror("fork");
        throw std::runtime_error("Unable to fork()");
    }

    if(_pid == 0)
    {
        ::execvp(argv[0], argv);
        perror("execvp");
        throw std::runtime_error("Unknown command");
    }

}

int Process::wait()
{
    int status;
    while(::waitpid(_pid, &status, 0) != _pid);
    printf("Child process '%s' exited with status %d\n", _name.c_str(), status);
    return status;

}

void Process::kill(int signal)
{
    ::kill(_pid, signal);
    printf("Killed process '%s' with signal %d\n", _name.c_str(), signal);
}

Process::~Process()
{
    kill(SIGINT);
    wait();
}

/**
 * @brief Function to be called in the main of each test, it runs roscore and fill
 * parameter server with robot models
 * 
 * @return a not 0 if some error happens
 */
int prepareROSForTests ( int argc, char **argv, std::string testName ) {
    

    
    ros::init ( argc, argv, testName );
        
    /////////////////////////// I cant manage to make this working, to wait the roscore
    //ros::Time::init();
    //while (!ros::master::check()) //wait for roscore to be ready
    //{
    //    std::cout << "waiting for roscore..." << std::endl;
    //    ros::Duration(0.2).sleep();
    //}
    ////////////////////////////////////////////////////////////////////////////////
    
    //fill ros param with file models, needed by moveit parserMoveIt
    std::string modelPathURDF = ROSEE::Utils::getPackagePath() + "configs/urdf/" + argv[1];
    std::string modelPathSRDF = ROSEE::Utils::getPackagePath() + "configs/srdf/" + argv[1];

    //Is there a better way to parse?
    std::ifstream urdf(modelPathURDF + ".urdf");
    std::ifstream srdf(modelPathSRDF + ".srdf");
    std::stringstream sUrdf, sSrdf;
    sUrdf << urdf.rdbuf();
    sSrdf << srdf.rdbuf();

    ros::param::set("robot_description" , sUrdf.str());
    ros::param::set("robot_description_semantic" , sSrdf.str());
    ros::param::set("robot_name", argv[1]);
    
    return 0;
}


} //namespace TestUtils

} //namespace ROSEE

#endif // TESTUTILS_H
