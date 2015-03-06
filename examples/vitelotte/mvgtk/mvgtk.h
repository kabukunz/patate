#ifndef _VITELOTTE_EXAMPLE_MVGTK_MVGTK_
#define _VITELOTTE_EXAMPLE_MVGTK_MVGTK_


#include <list>
#include <map>
#include <string>
#include <ostream>

#include <Patate/vitelotte.h>

#include "../common/vgMeshWithCurves.h"


typedef std::map<std::string, unsigned> ArgMap;

typedef VGMeshWithCurves Mesh;


int parseAttribSet(const std::string& attr);


struct GlobalOptions
{
    bool verbose;
};


class MvgtkCommand
{
public:
    virtual ~MvgtkCommand();

    virtual bool parseArgs(int argc, char** argv, int& argi);
    virtual bool execute(Mesh& mesh, const GlobalOptions* opts = 0) = 0;
};


class OutputCommand : public MvgtkCommand
{
public:
    virtual bool parseArgs(int argc, char** argv, int& argi);
    virtual bool execute(Mesh& mesh, const GlobalOptions* opts = 0);

private:
    std::string m_outFilename;
};


class Mvgtk
{
public:
    enum
    {
        GlobalHelp,
        GlobalVerbose
    };


public:
    Mvgtk();
    ~Mvgtk();

    template<typename T>
    void registerCommand(const std::string &name)
    {
        m_factories.insert(std::make_pair(name, new GenericFactory<T>()));
    }

    void printUsage(std::ostream& out, int exitCode = 127) const;

    bool parseArgs(int argc, char** argv);

    bool executeCommands();

    GlobalOptions opts;

private:
    struct CommandFactory
    {
        virtual ~CommandFactory();
        virtual MvgtkCommand* create() = 0;
    };

    template <typename T>
    struct GenericFactory : public CommandFactory
    {
        virtual MvgtkCommand* create() { return new T; }
    };

    typedef std::map<std::string, CommandFactory*> FactoryMap;
    typedef std::list<MvgtkCommand*> CommandList;

private:
    ArgMap m_argMap;
    FactoryMap m_factories;
    CommandList m_commands;
    std::string m_progName;
    std::string m_inFilename;
};


#endif
