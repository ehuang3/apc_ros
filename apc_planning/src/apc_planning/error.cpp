#include <ros/ros.h>
#include <apc_planning/error.h>
#include <boost/xpressive/xpressive.hpp>
// #define _GNU_SOURCE
#include <dlfcn.h>
#include <sstream>
// #include <stdio.h>
// #include <stdlib.h>
// #include <errno.h>
// #include <string.h>

#define BACKWARD_HAS_BFD 1
#include <backward.hpp>

std::string apc_exception::GetResolvedStackTrace()
{
    char *bp;
    size_t size;
    FILE *stream;
    stream = open_memstream (&bp, &size);

    // if (!stream) {
    //     ROS_INFO("%s", strerror(errno));
    //     return strerror(errno);
    // }

    using namespace backward;
    using namespace boost::xpressive;

    StackTrace st;
    st.load_here(12);
    TraceResolver tr; tr.load_stacktrace(st);
    Printer p;
    p.object = false;
    p.color = true;
    p.address = true;
    p.snippet = true;
    // p.print(st, stream);
    ResolvedTrace rt = tr.resolve(st[0]);

    Colorize c(stream);
    c.init();

    for (int i = 3; i < st.size(); i++) {
        ResolvedTrace trace = tr.resolve(st[i]);

        ResolvedTrace::SourceLoc source = trace.source;

        std::string object_filename = trace.object_filename;
        {
            sregex rex =
                sregex::compile("^([A-Za-z_/0-9]+/)([A-Za-z_]+\\.so)");
            smatch what;
            if (regex_match(object_filename, what, rex))
                object_filename = what[2];
        }
        trace.object_filename = object_filename;

        std::string object_function = trace.object_function;
        {
            sregex rex =
                sregex::compile("^([A-Za-z_:0-9]+::)([A-Za-z_0-9]+)(\\(.*\\))");
            smatch what;
            if (regex_match(object_function, what, rex))
                object_function = what[2];
        }
        trace.object_function = object_function;

        std::string source_filename = trace.source.filename;
        {
            sregex rex =
                sregex::compile("^([A-Za-z_/0-9]+/)([A-Za-z_]+\\.[a-z]+)");
            smatch what;
            if (regex_match(source_filename, what, rex))
                source_filename = what[2];
        }
        trace.source.filename = source_filename;

        std::string source_function = trace.source.function;
        {
            sregex rex =
                sregex::compile("^([A-Za-z_:0-9]+::)*([A-Za-z_0-9]+)(\\(.*\\))*");
            smatch what;
            if (regex_match(source_function, what, rex))
                source_function = what[2];
            else {
                ROS_DEBUG_STREAM(source_function);
                continue;
            }
        }
        trace.source.function = source_function;

        p.print_trace(stream, trace, c);

        p.print_snippet(stream, "      ", source, c, Color::yellow, 5);

        fflush (stream);
    }
    // for (size_t i = 1; i < st.size(); ++i) {
    //     ResolvedTrace trace = tr.resolve(st[i]);
    //     p.print(trace, stream);
    //     fflush (stream);
    // }
    fflush (stream);

    // printf("bp %p", bp);

    std::string out = bp;
    fclose(stream);
    free(bp);

    return out;
}

namespace apc_exception
{
        // void *trace[16];                                                \
        // char **messages = (char **)NULL;                                \
        // int i, trace_size = 0;                                          \
        // trace_size = backtrace(trace, 16);                              \
        // messages = backtrace_symbols(trace, trace_size);                \
        // printf("[bt] Execution path:\n");                               \
        // for (i=0; i<trace_size; ++i) {                                  \
        //     printf("[bt] %s\n", messages[i]);                           \
        //     int p = 0;                                                  \
        //     while(messages[i][p] != '(' && messages[i][p] != ' '        \
        //           && messages[i][p] != 0)                               \
        //         ++p;                                                    \
        //     char syscom[4096];                                          \
        //     sprintf(syscom,"addr2line %p -e %.*s", trace[i], p,         \
        //             messages[i]);                                       \
        //     printf("%s", syscom);                                       \
        //     system(syscom);                                             \
        // }                                                               \

/**
 * Execute a command and get the result.
 *
 * @param   cmd - The system command to run.
 * @return  The string command line output of the command.
 */
    std::string GetStdoutFromCommand(std::string cmd) {

        std::string data;
    FILE * stream;
    const int max_buffer = 256;
    char buffer[max_buffer];
    cmd.append(" 2>&1"); // Do we want STDERR?

    stream = popen(cmd.c_str(), "r");
    if (stream) {
        while (!feof(stream))
            if (fgets(buffer, max_buffer, stream) != NULL) data.append(buffer);
        pclose(stream);
    }
    return data;
}





    std::string show_backtrace()
    {
        void *trace[16];
        char **messages = (char **)NULL;
        int i, trace_size = 0;

        std::stringstream stream;

        trace_size = backtrace(trace, 16);
        messages = backtrace_symbols(trace, trace_size);

        stream << "[bt    ]\n";

        std::vector<int> fn_index_v;
        std::vector<std::string> fn_name_v;
        std::vector<std::string> fn_file_v;
        std::vector<std::string> fn_pos_v;

        // printf("[bt] Execution path:\n");
        for (i=1; i<trace_size; ++i) {
            // printf("[bt] %s\n", messages[i]);

            /* find first occurence of '(' or ' ' in message[i] and assume
             * everything before that is the file name. (Don't go beyond 0 though
             * (string terminator)*/
            size_t p = 0;
            while(messages[i][p] != '(' && messages[i][p] != ' '
                  && messages[i][p] != 0)
                ++p;

            Dl_info info;
            dladdr(trace[i], &info);

            // ROS_WARN("so@ %p", info.dli_fbase);
            size_t base_addr = (size_t) info.dli_fbase;

            using namespace boost::xpressive;
            sregex rex = sregex::compile("^([A-Za-z0-9_/\\-]+\\.so)(\\([A-Za-z0-9_]+\\+(0x[0-9a-f]+)\\)) (\\[(0x[0-9a-f]+)\\])");
            smatch what;
            std::string message = messages[i]; // Cannot pass temporary string to regex_match.
            // ROS_WARN_STREAM(message);
            if (regex_match(message, what, rex))
                for (int j = 0; j < what.size(); j++) {
                    // ROS_DEBUG_STREAM(what[j].str());
                }

            std::string fn_hex = what[5].str();
            size_t fn_addr = strtol(fn_hex.c_str(), NULL, 0);

            std::string off_hex = what[3].str();
            size_t fn_addr_off = strtol(off_hex.c_str(), NULL, 0);

            // ROS_ERROR("%s", what[2].str().c_str());
            // ROS_ERROR("%s", what[3].str().c_str());
            // ROS_ERROR("%#*lx", (int) fn_hex.length(), fn_addr - base_addr - fn_addr_off);

            char syscom[4096];
            sprintf(syscom,"addr2line -f -C %#*lx -e %.*s", (int) fn_hex.length(), fn_addr - base_addr,
                    (int)p, messages[i]);
            //last parameter is the file name of the symbol
            // printf("[system] %s\n", syscom);

            std::string bt = GetStdoutFromCommand(syscom);
            // ROS_DEBUG_STREAM(bt);

            sregex rex_2 =
                sregex::compile("^([A-Za-z_:]+::)([A-Za-z_]+)(\\(.*\\))\\n([A-Za-z0-9_/]+/)([A-Za-z_]+\\.[A-Za-z]+):([0-9]+).*\\n");
            smatch what_2;
            if (!regex_match(bt, what_2, rex_2)) {
                // ROS_ERROR_STREAM("Failed to parse " << bt);
                continue;
            }


            std::string fn_name = what_2[2];
            std::string fn_file = what_2[5];
            std::string fn_pos  = what_2[6];

            // stream << "[bt] " << fn_name << std::endl;
            // stream << "     " << fn_file << ":" << fn_pos << std::endl;

            // stream << "[bt] " << fn_name << std::endl;
            // stream << "     " << fn_file << ":" << fn_pos << std::endl;

            fn_index_v.push_back(i);
            fn_name_v.push_back(fn_name);
            fn_file_v.push_back(fn_file);
            fn_pos_v.push_back(fn_pos);

            // system(syscom);
        }

        int int_len = 0;
        int name_len = 0;
        int file_len = 0;
        int pos_len = 0;
        for (int i = 0; i < fn_name_v.size(); i++) {
            if (name_len < fn_name_v[i].size())
                name_len = fn_name_v[i].size();
            if (file_len < fn_file_v[i].size())
                file_len = fn_file_v[i].size();
            if (pos_len < fn_pos_v[i].size())
                pos_len = fn_pos_v[i].size();
        }

        char buf[4096];
        for (int i = 0; i < fn_name_v.size(); i++) {
            sprintf(buf, "[bt #%2d] %*s()  %*s  %*s\n", fn_index_v[i], name_len, fn_name_v[i].c_str(),
                    file_len, fn_file_v[i].c_str(), pos_len, fn_pos_v[i].c_str());
            stream << buf;
                }

        return stream.str();
    }
}


// [bt] /home/ehuang/catkin_ws/devel/lib/libmoveit_background_processing.so(_ZNK5boost4_mfi3mf0IvN6moveit5tools20BackgroundProcessingEEclEPS4_+0x65) [0x7fff984776cd]
// ??:0
// addr2line 0x7fff984776cd -e /home/ehuang/catkin_ws/devel/lib/libmoveit_background_processing.so

// /home/ehuang/catkin_ws/devel/lib/libmoveit_motion_planning_rviz_plugin_core.so(_ZN18moveit_rviz_plugin19MotionPlanningFrame24computeReachableBinPosesERSt6vectorIN8apc_msgs14PrimitivePlan_ISaIvEEESaIS5_EERKSsRKN6moveit4core10RobotStateERKSt3mapISsN5Eigen9TransformIdLi3ELi2ELi0EEESt4lessISsENSH_17aligned_allocatorISt4pairIS9_SJ_EEEE+0x31b) [0x7fff9abd555b]
