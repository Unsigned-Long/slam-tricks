#pragma once

/**
 * @file logger.h
 * @author csl (3079625093@qq.com)
 * @version 0.1
 * @date 2022-01-27
 *
 * @copyright Copyright (c) 2022
 */

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

/**
 *
 * @brief the main message type macros
 * [0] LOG_PLAINTEXT {the plain text}
 * [1] LOG_INFO    {Information; Message; real-time info Of information; Of messages; Informative}
 * [2] LOG_PROCESS {The process of achieving a goal; The development of things, especially the steps of natural change;}
 * [3] LOG_WARNING {about possible accidents, etc.; a warning, warning, etc about the punishment to be suffered}
 * [4] LOG_ERROR   {Error; Errors; Fallacy;}
 * [5] LOG_FATAL   {Fatal; Catastrophic; Destructive; Cause failure}
 *
 * @brief debug macroes
 * [1] LOG_VAR print the variables
 * [2] LOG_ENDL print an end line char '\n'

 * @brief the macros to control the using of the STL containers output format
 * [1] FORMAT_MAP FORMAT_MULTIMAP FORMAT_UNORDERED_MAP FORMAT_UNORDERED_MULTIMAP
 * [2] FORMAT_SET FORMAT_UNORDERED_SET FORMAT_MULTISET FORMAT_UNORDERED_MULTISET
 * [3] FORMAT_VECTOR FORMAT_LIST FORMAT_DEQUE FORMAT_ARRAY
 * [4] FORMAT_STACK FORMAT_QUEUE
 *
 * @brief time output format
 * [1] FORMAT_TIME_HMS hh:mm:ss (default mode)
 * [2] FORMAT_TIME_STAMP time stamp since epoch
 */

#pragma region output for container declare

/**
 * @brief overload the operator '<<' for std::pair
 */
template <typename Key, typename Val>
std::ostream &operator<<(std::ostream &os, const std::pair<Key, Val> &p);

/**
 * @brief output format for container
 */
template <typename ConType>
std::ostream &orderedContainer(std::ostream &os, const ConType &s);

/**
 * @brief output format for unordered container
 */
template <typename ConType>
std::ostream &unorderedContainer(std::ostream &os, const ConType &c);

#pragma endregion

#pragma region map

#ifdef FORMAT_MAP
#include <map>
/**
 * @brief overload the operator '<<' for std::map
 */
template <typename Key, typename Val>
std::ostream &operator<<(std::ostream &os, const std::map<Key, Val> &m) {
  return orderedContainer(os, m);
}
#endif

#ifdef FORMAT_MULTIMAP
#include <map>
/**
 * @brief overload the operator '<<' for std::multimap
 */
template <typename Key, typename Val>
std::ostream &operator<<(std::ostream &os, const std::multimap<Key, Val> &m) {
  return orderedContainer(os, m);
}
#endif

#ifdef FORMAT_UNORDERED_MAP

#include <unordered_map>

/**
 * @brief overload the operator '<<' for std::unordered_map
 */
template <typename Key, typename Val>
std::ostream &operator<<(std::ostream &os,
                         const std::unordered_map<Key, Val> &m) {
  return unorderedContainer(os, m);
}

#endif

#ifdef FORMAT_UNORDERED_MULTIMAP
#include <unordered_map>
/**
 * @brief overload the operator '<<' for std::unordered_multimap
 */
template <typename Key, typename Val>
std::ostream &operator<<(std::ostream &os,
                         const std::unordered_multimap<Key, Val> &m) {
  return unorderedContainer(os, m);
}
#endif

#pragma endregion

#pragma region set

#ifdef FORMAT_SET
#include <set>
/**
 * @brief overload the operator '<<' for std::set
 */
template <typename Val>
std::ostream &operator<<(std::ostream &os, const std::set<Val> &s) {
  return orderedContainer(os, s);
}
#endif

#ifdef FORMAT_UNORDERED_SET
#include <unordered_set>
/**
 * @brief overload the operator '<<' for std::unordered_set
 */
template <typename Val>
std::ostream &operator<<(std::ostream &os, const std::unordered_set<Val> &s) {
  return unorderedContainer(os, s);
}
#endif

#ifdef FORMAT_MULTISET
#include <set>
/**
 * @brief overload the operator '<<' for std::multiset
 */
template <typename Val>
std::ostream &operator<<(std::ostream &os, const std::multiset<Val> &s) {
  return orderedContainer(os, s);
}
#endif

#ifdef FORMAT_UNORDERED_MULTISET
#include <unordered_set>
/**
 * @brief overload the operator '<<' for std::unordered_multiset
 */
template <typename Val>
std::ostream &operator<<(std::ostream &os,
                         const std::unordered_multiset<Val> &s) {
  return unorderedContainer(os, s);
}
#endif
#pragma endregion

#pragma region seq cons

#ifdef FORMAT_VECTOR

#include <vector>

/**
 * @brief overload the operator '<<' for std::vector
 */
template <typename Val>
std::ostream &operator<<(std::ostream &os, const std::vector<Val> &s) {
  return orderedContainer(os, s);
}

#endif

#ifdef FORMAT_LIST
#include <list>
/**
 * @brief overload the operator '<<' for std::list
 */
template <typename Val>
std::ostream &operator<<(std::ostream &os, const std::list<Val> &s) {
  return orderedContainer(os, s);
}
#endif

#ifdef FORMAT_DEQUE
#include <deque>
/**
 * @brief overload the operator '<<' for std::deque
 */
template <typename Val>
std::ostream &operator<<(std::ostream &os, const std::deque<Val> &s) {
  return orderedContainer(os, s);
}
#endif

#ifdef FORMAT_ARRAY
#include <array>
/**
 * @brief overload the operator '<<' for std::array
 */
template <typename Val, std::size_t Size>
std::ostream &operator<<(std::ostream &os, const std::array<Val, Size> &s) {
  os << '[';
  for (int i = 0; i != s.size() - 1; ++i)
    os << s[i] << ns_log::ns_priv::splitor;
  os << s.back() << ']';
  return os;
}
#endif

#ifdef FORMAT_STACK
#include <stack>
/**
 * @brief overload the operator '<<' for std::stack
 */
template <typename Val>
std::ostream &operator<<(std::ostream &os, const std::stack<Val> &s) {
  if (s.empty()) {
    os << "[]";
    return os;
  }
  os << "['top' ";
  auto cs = s;
  while (cs.size() != 1) {
    os << cs.top() << ns_log::ns_priv::splitor;
    cs.pop();
  }
  os << cs.top() << "]";
  return os;
}
#endif

#ifdef FORMAT_QUEUE
#include <queue>
/**
 * @brief overload the operator '<<' for std::queue
 */
template <typename Val>
std::ostream &operator<<(std::ostream &os, const std::queue<Val> &q) {
  if (q.empty()) {
    os << "[]";
    return os;
  }
  os << "['front' ";
  auto cq = q;
  while (cq.size() != 1) {
    os << cq.front() << ns_log::ns_priv::splitor;
    cq.pop();
  }
  os << cq.front() << "]";
  return os;
}
#endif

#pragma endregion

namespace ns_log {

  namespace ns_priv {

#define LOG_STYLE_NONE std::string("\033[0m")
#define LOG_STYLE_BOLD std::string("\033[1m")
#define LOG_STYLE_ITALIC std::string("\033[3m")
#define LOG_STYLE_INFO std::string("\033[92m")
#define LOG_STYLE_PROCESS std::string("\033[94m")
#define LOG_STYLE_ERROR std::string("\033[91m")
#define LOG_STYLE_WARNING std::string("\033[93m")
#define LOG_STYLE_FATAL std::string("\033[95m")

// the prefix used when print variables
#define LOG_PREFIX "-- "

// the suffix used when print variables
#define LOG_SUFFIX ""

    /**
     * @brief base logger
     */
    class Logger {
    public:
      /**
       * @brief the members
       */
      std::ostream *_loggerOS;
      int _precision;

    public:
      /**
       * @brief construct a new Logger object
       */
      explicit Logger(std::ostream *os) : _loggerOS(os), _precision(5) {}

      virtual ~Logger() = default;

      Logger *setPrecision(int n) {
        (*this->_loggerOS) << std::fixed << std::setprecision(n);
        _precision = n;
        return this;
      }

      template <typename... ArgsType>
      Logger &operator()(const std::string &desc, const std::string &color, const ArgsType &...args) {
        std::stringstream stream;
        stream << std::fixed << std::setprecision(_precision);
        Logger::_print_(stream, args...);
        *(this->_loggerOS) << this->getMessageHeader(desc, color) << ' '
                           << this->getMessage(stream.str(), color);
        return *this;
      }

      template <typename... ArgsType>
      Logger &plaintext(const ArgsType &...args) {
        std::stringstream stream;
        stream << std::fixed << std::setprecision(_precision);
        Logger::_print_(stream, LOG_PREFIX, args...);
        *(this->_loggerOS) << this->getMessage(stream.str(), "");
        return *this;
      }

      template <typename... ArgsType>
      Logger &info(const ArgsType &...args) {
        (*this)("info", LOG_STYLE_INFO, args...);
        return *this;
      }

      template <typename... ArgsType>
      Logger &warning(const ArgsType &...args) {
        (*this)("warning", LOG_STYLE_WARNING, args...);
        return *this;
      }

      template <typename... ArgsType>
      Logger &process(const ArgsType &...args) {
        (*this)("process", LOG_STYLE_PROCESS, args...);
        return *this;
      }

      template <typename... ArgsType>
      Logger &fatal(const ArgsType &...args) {
        (*this)("fatal", LOG_STYLE_FATAL, args...);
        return *this;
      }

      template <typename... ArgsType>
      Logger &error(const ArgsType &...args) {
        (*this)("error", LOG_STYLE_ERROR, args...);
        return *this;
      }

      virtual std::string getMessageHeader(const std::string &desc, const std::string &color) = 0;

      virtual std::string getMessage(const std::string &msg, const std::string &color) = 0;

    protected:
      Logger &_print_(std::ostream &os) {
        os << '\n';
        return *this;
      }

      template <typename ArgType, typename... ArgsType>
      Logger &_print_(std::ostream &os, const ArgType &arg, const ArgsType &...args) {
        os << arg;
        Logger::_print_(os, args...);
        return *this;
      }

      /**
       * @brief get the time when the message is outputted
       *
       * @return double
       */
      static std::string curTime() {
#ifdef FORMAT_TIME_HMS
        const time_t t = time(NULL);
        auto time = localtime(&t);
        return std::to_string(time->tm_hour) + ':' +
               std::to_string(time->tm_min) + ':' +
               std::to_string(time->tm_sec);
#elif defined(FORMAT_TIME_STAMP)
        auto now = std::chrono::system_clock::now();
        auto sed = std::chrono::time_point_cast<std::chrono::duration<double>>(now)
                       .time_since_epoch()
                       .count();
        return std::to_string(sed) + "(S)";
#else
        const time_t t = time(NULL);
        auto time = localtime(&t);
        return std::to_string(time->tm_hour) + ':' +
               std::to_string(time->tm_min) + ':' +
               std::to_string(time->tm_sec);
#endif
      }

    public:
      Logger() = delete;

      Logger(const Logger &) = delete;

      Logger(Logger &&) = delete;

      Logger &operator=(const Logger &) = delete;

      Logger &operator=(Logger &&) = delete;
    };

  } // namespace ns_priv

  class StdLogger : public ns_priv::Logger {

  public:
    explicit StdLogger(std::ostream &os) : Logger(&os) {}

    ~StdLogger() override = default;

    std::string getMessageHeader(const std::string &desc, const std::string &color) override {
#ifdef __linux__
      auto flag = '[' + LOG_STYLE_BOLD + color + desc + LOG_STYLE_NONE + ']';
      auto tm = '[' + LOG_STYLE_ITALIC + color + Logger::curTime() + LOG_STYLE_NONE + ']';
      return flag + '-' + tm;
#else
      return '[' + desc + "]-[" + Logger::curTime() + "]";
#endif
    }

    std::string getMessage(const std::string &msg, const std::string &color) override {
#ifdef __linux__
      return LOG_STYLE_ITALIC + color + msg + LOG_STYLE_NONE;
#else
      return msg;
#endif
    }
  };

  class FileLogger : public ns_priv::Logger {
  public:
    explicit FileLogger(const std::string &filename) : Logger(new std::ofstream(filename, std::ios::out)) {}

    ~FileLogger() override {
      dynamic_cast<std::ofstream *>(this->_loggerOS)->close();
      delete this->_loggerOS;
    }

    std::string getMessageHeader(const std::string &desc, const std::string &color) override {
      return '[' + desc + "]-[" + Logger::curTime() + "]";
    }

    std::string getMessage(const std::string &msg, const std::string &color) override {
      return msg;
    }
  };

#undef LOG_STYLE_NONE
#undef LOG_STYLE_BOLD
#undef LOG_STYLE_ITALIC
#undef LOG_STYLE_INFO
#undef LOG_STYLE_PROCESS
#undef LOG_STYLE_ERROR
#undef LOG_STYLE_WARNING
#undef LOG_STYLE_FATAL

  namespace ns_priv {
    /**
     * @brief params to control
     * @param _splitor_ the splitor to split the elements
     * @param _firName_ the describe name for the first element of the std::pair
     * @param _sedName_ the describe name for the second element of the std::pair
     */
    static const std::string splitor(", ");

    static StdLogger stdCoutLogger(std::cout);
  } // namespace ns_priv

  template <typename... ArgsType>
  static ns_priv::Logger &plaintext(const ArgsType &...args) {
    return ns_log::ns_priv::stdCoutLogger.plaintext(args...);
  }

#define LOG_PLAINTEXT(...) \
  ns_log::ns_priv::stdCoutLogger.plaintext(__VA_ARGS__);
#define LOG_PLAINTEXT_F(flogger, ...) \
  flogger.plaintext(__VA_ARGS__);

  template <typename... ArgsType>
  static ns_priv::Logger &info(const ArgsType &...args) {
    return ns_log::ns_priv::stdCoutLogger.info(args...);
  }

#define LOG_INFO(...) \
  ns_log::ns_priv::stdCoutLogger.info(__VA_ARGS__);
#define LOG_INFO_F(flogger, ...) \
  flogger.info(__VA_ARGS__);

  template <typename... ArgsType>
  static ns_priv::Logger &process(const ArgsType &...args) {
    return ns_log::ns_priv::stdCoutLogger.process(args...);
  }

#define LOG_PROCESS(...) \
  ns_log::ns_priv::stdCoutLogger.process(__VA_ARGS__);
#define LOG_PROCESS_F(flogger, ...) \
  flogger.process(__VA_ARGS__);

  template <typename... ArgsType>
  static ns_priv::Logger &warning(const ArgsType &...args) {
    return ns_log::ns_priv::stdCoutLogger.warning(args...);
  }

#define LOG_WARNING(...) \
  ns_log::ns_priv::stdCoutLogger.warning(__VA_ARGS__);
#define LOG_WARNING_F(flogger, ...) \
  flogger.warning(__VA_ARGS__);

  template <typename... ArgsType>
  static ns_priv::Logger &error(const ArgsType &...args) {
    return ns_log::ns_priv::stdCoutLogger.error(args...);
  }

#define LOG_ERROR(...) \
  ns_log::ns_priv::stdCoutLogger.error(__VA_ARGS__);
#define LOG_ERROR_F(flogger, ...) \
  flogger.error(__VA_ARGS__);

  template <typename... ArgsType>
  static ns_priv::Logger &fatal(const ArgsType &...args) {
    return ns_log::ns_priv::stdCoutLogger.fatal(args...);
  }

#define LOG_FATAL(...) \
  ns_log::ns_priv::stdCoutLogger.fatal(__VA_ARGS__);
#define LOG_FATAL_F(flogger, ...) \
  flogger.fatal(__VA_ARGS__);

// a macro launcher
#define MACRO_SELFT(X) X

#define MACRO_VAR_ARGS_IMPL_COUNT(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, N, ...) N
#define COUNT_MACRO_VAR_ARGS(...) MACRO_SELFT(MACRO_VAR_ARGS_IMPL_COUNT(__VA_ARGS__, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0))

#define MACRO_COMBINE_2(MACRO, ARGS_COUNT) MACRO##ARGS_COUNT
#define MACRO_COMBINE_1(MACRO, ARGS_COUNT) MACRO_COMBINE_2(MACRO, ARGS_COUNT)
#define MACRO_COMBINE(MACRO, ARGS_COUNT) MACRO_COMBINE_1(MACRO, ARGS_COUNT)

#define MACRO_LAUNCHER(MACRO, ...) \
  MACRO_SELFT(MACRO_COMBINE(MACRO, COUNT_MACRO_VAR_ARGS(__VA_ARGS__))(__VA_ARGS__))

// a helper
#define _LOG_VAR_PACK_(var) \
#var << ": " << var << ", "

#define _LOG_VAR_1(var) #var << ": " << var
#define _LOG_VAR_2(var, ...) _LOG_VAR_PACK_(var) << _LOG_VAR_1(__VA_ARGS__)
#define _LOG_VAR_3(var, ...) _LOG_VAR_PACK_(var) << _LOG_VAR_2(__VA_ARGS__)
#define _LOG_VAR_4(var, ...) _LOG_VAR_PACK_(var) << _LOG_VAR_3(__VA_ARGS__)
#define _LOG_VAR_5(var, ...) _LOG_VAR_PACK_(var) << _LOG_VAR_4(__VA_ARGS__)
#define _LOG_VAR_6(var, ...) _LOG_VAR_PACK_(var) << _LOG_VAR_5(__VA_ARGS__)
#define _LOG_VAR_7(var, ...) _LOG_VAR_PACK_(var) << _LOG_VAR_6(__VA_ARGS__)
#define _LOG_VAR_8(var, ...) _LOG_VAR_PACK_(var) << _LOG_VAR_7(__VA_ARGS__)
#define _LOG_VAR_9(var, ...) _LOG_VAR_PACK_(var) << _LOG_VAR_8(__VA_ARGS__)
#define _LOG_VAR_10(var, ...) _LOG_VAR_PACK_(var) << _LOG_VAR_9(__VA_ARGS__)

// print variables for debug or something else
#define LOG_VAR(...)                                                             \
  std::cout << LOG_PREFIX << "\033[3m" << MACRO_LAUNCHER(_LOG_VAR_, __VA_ARGS__) \
            << LOG_SUFFIX << "\033[0m" << std::endl;

// print var value to a file
#define LOG_VAR_F(flogger, ...)                                                \
  *(flogger._loggerOS) << LOG_PREFIX << MACRO_LAUNCHER(_LOG_VAR_, __VA_ARGS__) \
                       << LOG_SUFFIX << std::endl;

#define LOG_ENDL() \
  std::cout << std::endl;

#define LOG_ENDL_F(flogger) \
  *(flogger._loggerOS) << std::endl;

} // namespace ns_log

#pragma region output for container

/**
 * @brief overload the operator '<<' for std::pair
 */
template <typename Key, typename Val>
std::ostream &operator<<(std::ostream &os, const std::pair<Key, Val> &p) {
  os << "{'" << p.first << "': " << p.second << '}';
  return os;
}

/**
 * @brief output format for container
 */
template <typename ConType>
std::ostream &orderedContainer(std::ostream &os, const ConType &s) {
  os << '[';
  if (s.empty()) {
    os << "]";
    return os;
  }
  auto iter = s.cbegin();
  for (; iter != (--s.cend()); ++iter)
    os << *iter << ns_log::ns_priv::splitor;
  os << *iter << ']';
  return os;
}

/**
 * @brief output format for unordered container
 */
template <typename ConType>
std::ostream &unorderedContainer(std::ostream &os, const ConType &c) {
  os << '[';
  if (c.empty()) {
    os << "]";
    return os;
  }
  std::stringstream stream;
  for (const auto &elem : c)
    stream << elem << ns_log::ns_priv::splitor;
  std::string str = stream.str();
  os << std::string_view(str.c_str(),
                         str.size() - ns_log::ns_priv::splitor.size())
     << ']';
  return os;
}

#pragma endregion