// Copyright 2011-2013 Paul Furgale and others, 2017, 2019 ETH Zürich, Thomas Schöps
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include "rrt_planner/timing.h"
#include <iostream>
#include <sstream>
#include <algorithm>
#include <limits>
#include <map>
#include <math.h>

Timer::Timer(bool construct_stopped)
    : timing_(false),
      handle_(std::numeric_limits<std::size_t>::max()) {
  if (!construct_stopped) {
    Start();
  }
}

Timer::Timer(std::size_t handle, bool construct_stopped)
    : timing_(false),
      handle_(handle) {
  if (!construct_stopped) {
    Start();
  }
}

Timer::Timer(const std::string& tag, bool construct_stopped)
    : timing_(false),
      handle_(Timing::getHandle(tag)) {
  if (!construct_stopped) {
    Start();
  }
}

Timer::Timer(const char* tag, bool construct_stopped)
    : timing_(false),
      handle_(Timing::getHandle(tag)) {
  if (!construct_stopped) {
    Start();
  }
}

Timer::~Timer() {
  if (IsTiming()) {
    Stop();
  }
}

void Timer::Start() {
  timing_ = true;
  start_time_ = std::chrono::steady_clock::now();
}

double Timer::Stop(bool add_to_statistics) {
  double seconds = GetTimeSinceStart();
  if (add_to_statistics && handle_ != std::numeric_limits<std::size_t>::max()) {
    Timing::addTime(handle_, seconds);
  }
  timing_ = false;
  return seconds;
}

double Timer::GetTimeSinceStart() {
  std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
  double seconds = 1e-9 * std::chrono::duration<double, std::nano>(now - start_time_).count();
  return seconds;
}

// Algorithm from:
// https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Online_algorithm
struct TimerMapValue {
  TimerMapValue()
      : count(0),
        min(std::numeric_limits<double>::infinity()),
        max(-std::numeric_limits<double>::infinity()),
        M2(0),
        mean(0) {}
  
  void AddValue(double x) {
    count += 1;
    double delta = x - mean;
    mean += delta / count;
    double delta2 = x - mean;
    M2 += delta * delta2;
    
    if (x < min) {
      min = x;
    }
    if (x > max) {
      max = x;
    }
  }
  
  double GetVariance() const {
    if (count < 2) {
      return 0;
    } else {
      return M2 / (count - 1);
    }
  }
  
  double GetTotal() const {
    return count * mean;
  }
  
  std::size_t count;
  double min;
  double max;
  double M2;
  double mean;
};


std::mutex Timing::m_mutex;

Timing& Timing::instance() {
  static Timing t;
  return t;
}

Timing::Timing() :
    m_maxTagLength(0) {}

Timing::~Timing() {}

std::size_t Timing::getHandle(std::string const& tag){
  // Search for an existing tag.
  std::unique_lock<std::mutex> lock(m_mutex);
  map_t::iterator i = instance().m_tagMap.find(tag);
  if (i == instance().m_tagMap.end()) {
    // If it is not there, create a tag.
    std::size_t handle = instance().m_timers.size();
    instance().m_tagMap[tag] = handle;
    instance().m_timers.push_back(TimerMapValue());
    // Track the maximum tag length to help printing a table of timing values later.
    instance().m_maxTagLength = std::max(instance().m_maxTagLength, tag.size());
    return handle;
  } else {
    return i->second;
  }
}

std::string Timing::getTag(std::size_t handle){
  std::string tag;
  bool found = false;
  
  // Perform a linear search for the tag
  map_t::iterator i = instance().m_tagMap.begin();
  for ( ; i != instance().m_tagMap.end(); i++) {
    if (i->second == handle){
      found = true;
      tag = i->first;
      break;
    }
  }
  return tag;
}

void Timing::addTime(std::size_t handle, double seconds) {
  std::unique_lock<std::mutex> lock(m_mutex);
  instance().m_timers[handle].AddValue(seconds);
}

double Timing::getTotalSeconds(std::size_t handle) {
  // CHECK_LT(handle, instance().m_timers.size()) << "Handle is out of range: " << handle << ", number of timers: " << instance().m_timers.size();
  return instance().m_timers[handle].GetTotal();
}

double Timing::getTotalSeconds(std::string const& tag) {
  return getTotalSeconds(getHandle(tag));
}

double Timing::getMeanSeconds(std::size_t handle) {
  // CHECK_LT(handle, instance().m_timers.size()) << "Handle is out of range: " << handle << ", number of timers: " << instance().m_timers.size();
  return instance().m_timers[handle].mean;
}

double Timing::getMeanSeconds(std::string const& tag) {
  return getMeanSeconds(getHandle(tag));
}

std::size_t Timing::getNumSamples(std::size_t handle) {
  // CHECK_LT(handle, instance().m_timers.size()) << "Handle is out of range: " << handle << ", number of timers: " << instance().m_timers.size();
  return instance().m_timers[handle].count;
}

std::size_t Timing::getNumSamples(std::string const& tag) {
  return getNumSamples(getHandle(tag));
}

double Timing::getVarianceSeconds(std::size_t handle) {
  //CHECK_LT(handle, instance().m_timers.size()) << "Handle is out of range: " << handle << ", number of timers: " << instance().m_timers.size();
  return instance().m_timers[handle].GetVariance();
}

double Timing::getVarianceSeconds(std::string const& tag) {
  return getVarianceSeconds(getHandle(tag));
}

double Timing::getMinSeconds(std::size_t handle) {
  //CHECK_LT(handle, instance().m_timers.size()) << "Handle is out of range: " << handle << ", number of timers: " << instance().m_timers.size();
  return instance().m_timers[handle].min;
}

double Timing::getMinSeconds(std::string const& tag) {
  return getMinSeconds(getHandle(tag));
}

double Timing::getMaxSeconds(std::size_t handle) {
  //CHECK_LT(handle, instance().m_timers.size()) << "Handle is out of range: " << handle << ", number of timers: " << instance().m_timers.size();
  return instance().m_timers[handle].max;
}

double Timing::getMaxSeconds(std::string const& tag) {
  return getMaxSeconds(getHandle(tag));
}

double Timing::getHz(std::size_t handle) {
  //CHECK_LT(handle, instance().m_timers.size()) << "Handle is out of range: " << handle << ", number of timers: " << instance().m_timers.size();
  return 1.0 / instance().m_timers[handle].mean;
}

double Timing::getHz(std::string const& tag) {
  return getHz(getHandle(tag));
}

void Timing::reset(std::size_t handle) {
  std::unique_lock<std::mutex> lock(m_mutex);
  // CHECK_LT(handle, instance().m_timers.size()) << "Handle is out of range: " << handle << ", number of timers: " << instance().m_timers.size();
  instance().m_timers[handle] = TimerMapValue();
}

void Timing::reset(std::string const& tag) {
  return reset(getHandle(tag));
}

std::string Timing::secondsToTimeString(double seconds, bool long_format) {
  
//   double secs = fmod(seconds,60);
//   int minutes = (long)(seconds/60);
//   int hours = (long)(seconds/3600);
//   minutes = minutes - (hours*60);
//   
//   char buffer[256];
// #ifdef WIN32
//   sprintf_s(buffer,256,"%02d:%02d:%09.6f",hours,minutes,secs);
// #else
//   sprintf(buffer,"%02d:%02d:%09.6f",hours,minutes,secs);
// #endif
  
  char buffer[256];
#ifdef WIN32
  sprintf_s(buffer, 256, long_format ? "%011.4f" : "%09.6f", seconds);
#else
  sprintf(buffer, long_format ? "%011.4f" : "%09.6f", seconds);
#endif
  return buffer;
}

template <typename TMap, typename Accessor>
void Timing::print(const TMap& tagMap, const Accessor& accessor, std::ostream& out) {
  out << "Timing\n";
  out << "------\n";
  for (typename TMap::const_iterator t = tagMap.begin(); t != tagMap.end(); ++ t) {
    std::size_t i = accessor.getIndex(t);
    if (getNumSamples(i) == 0) {
      continue;
    }
    
    out.width((std::streamsize)instance().m_maxTagLength);
    out.setf(std::ios::left,std::ios::adjustfield);
    out << accessor.getTag(t) << "\t";
    
    out.width(8);
    out.setf(std::ios::right,std::ios::adjustfield);
    out << getNumSamples(i) << "\t";
    if (getNumSamples(i) > 0) {
      out << secondsToTimeString(getTotalSeconds(i), true) << "\t";
      double meansec = getMeanSeconds(i);
      double stddev = sqrt(getVarianceSeconds(i));
      out << "(" << secondsToTimeString(meansec) << " +- ";
      out << secondsToTimeString(stddev) << ")\t";

      double minsec = getMinSeconds(i);
      double maxsec = getMaxSeconds(i);

      // The min or max are out of bounds.
      out << "[" << secondsToTimeString(minsec) << "," << secondsToTimeString(maxsec) << "]";

    }
    out << std::endl;
  }
}

void Timing::print(std::ostream& out) {
  struct Accessor {
    std::size_t getIndex(map_t::const_iterator t) const {
      return t->second;
    }
    const std::string&  getTag(map_t::const_iterator t) const {
      return t->first;
    }
  };

  print(instance().m_tagMap, Accessor(), out);
}

void Timing::print(std::ostream& out, const SortType sort) {
  map_t& tagMap = instance().m_tagMap;

  typedef std::multimap<double, std::string, std::greater<double> > SortMap_t;
  SortMap_t sorted;
  for(map_t::const_iterator t = tagMap.begin(); t != tagMap.end(); t++) {
    std::size_t i = t->second;
    double sv;
    if(getNumSamples(i) > 0)
      switch (sort) {
        case kSortByTotal:
          sv = getTotalSeconds(i);
          break;
        case kSortByMean:
          sv = getMeanSeconds(i);
          break;
        case kSortByStd:
          sv = sqrt(getVarianceSeconds(i));
          break;
        case kSortByMax:
          sv = getMaxSeconds(i);
          break;
        case kSortByMin:
          sv = getMinSeconds(i);
          break;
        case kSortByNumSamples:
          sv = getNumSamples(i);
          break;
      }
    else
        sv = std::numeric_limits<double>::max();
    sorted.insert(SortMap_t::value_type(sv, t->first));
  }

  struct Accessor {
    map_t& tagMap;
    Accessor(map_t& tagMap) : tagMap(tagMap) {}
    
    std::size_t getIndex(SortMap_t::const_iterator t) const {
      return tagMap[t->second];
    }
    const std::string& getTag(SortMap_t::const_iterator t) const {
      return t->second;
    }
  };
  
  print(sorted, Accessor(tagMap), out);
}

std::string Timing::print()
{
  std::stringstream ss;
  print(ss);
  return ss.str();
}

std::string Timing::print(const SortType sort)
{
  std::stringstream ss;
  print(ss, sort);
  return ss.str();
}

