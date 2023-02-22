#ifndef SIRE_LOG_HPP_
#define SIRE_LOG_HPP_

#include <string>

#include <aris/core/log.hpp>

#define SIRE_LOG aris::core::cout()

#define SIRE_DEBUG_LOG \
  SIRE_LOG << std::string(__FILE__) << "_" << std::to_string(__LINE__) << ": "

#endif