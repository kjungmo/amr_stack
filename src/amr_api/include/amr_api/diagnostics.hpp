// SPDX-License-Identifier: Apache-2.0
// Diagnostics seams owned by core: a logging sink and a clock. amr_api defines
// the abstract seams plus a no-op logger so modules/adapters run standalone
// (e.g. in gtests) without a core present.
#pragma once

#include <string>

namespace amr_api {

class Logger {
 public:
  enum class Level { Debug, Info, Warn, Error };
  virtual ~Logger() = default;
  virtual void log(Level level, const std::string& msg) = 0;
};

/// Default no-op logger (used when no core is wired in).
class NullLogger : public Logger {
 public:
  void log(Level /*level*/, const std::string& /*msg*/) override {}
};

class Clock {
 public:
  virtual ~Clock() = default;
  virtual double now() const = 0;  // seconds
};

}  // namespace amr_api
