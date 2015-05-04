/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _MVGTK_MVGTK_
#define _MVGTK_MVGTK_


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
    int repeat;
};


class MvgtkCommand
{
public:
    virtual ~MvgtkCommand();

    virtual bool parseArgs(int argc, char** argv, int& argi);
    virtual bool execute(Mesh& mesh, const GlobalOptions* opts = 0) = 0;

    inline static const char* cmdOptions() { return "Todo."; }
    inline static const char* cmdDesc() { return "Todo."; }
};


class OutputCommand : public MvgtkCommand
{
public:
    virtual bool parseArgs(int argc, char** argv, int& argi);
    virtual bool execute(Mesh& mesh, const GlobalOptions* opts = 0);

    static const char* cmdOptions();
    static const char* cmdDesc();

private:
    std::string m_outFilename;
};


class Mvgtk
{
public:
    enum GlobalParams
    {
        GlobalHelp,
        GlobalVerbose,
        GlobalRepeat
    };


public:
    Mvgtk();
    ~Mvgtk();

    template<typename T>
    void registerCommand(const std::string& name)
    {
        CommandFactory* fact = new GenericFactory<T>();
        m_factories.push_back(fact);
        m_factoryMap.insert(std::make_pair(name, fact));
    }
    void addCommandAlias(const std::string& command, const std::string& alias);

    void printUsage(std::ostream& out, int exitCode = 127) const;

    bool parseArgs(int argc, char** argv);

    bool executeCommands();

    GlobalOptions opts;

private:
    struct CommandFactory
    {
        virtual ~CommandFactory();
        virtual MvgtkCommand* create() = 0;
        virtual const char* cmdOptions() = 0;
        virtual const char* cmdDesc() = 0;
    };

    template <typename T>
    struct GenericFactory : public CommandFactory
    {
        virtual MvgtkCommand* create() { return new T; }
        virtual const char* cmdOptions() { return T::cmdOptions(); }
        virtual const char* cmdDesc() { return T::cmdDesc(); }
    };

    typedef std::vector<CommandFactory*> FactoryList;
    typedef std::map<std::string, CommandFactory*> FactoryMap;
    typedef std::vector<MvgtkCommand*> CommandList;

private:
    ArgMap m_argMap;
    FactoryList m_factories;
    FactoryMap m_factoryMap;
    CommandList m_commands;
    std::string m_progName;
    std::string m_inFilename;
};


#endif
