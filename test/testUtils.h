#ifndef TESTUTILS_H
#define TESTUTILS_H


#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/wait.h>

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

} //namespace TestUtils

} //namespace ROSEE

#endif // TESTUTILS_H
