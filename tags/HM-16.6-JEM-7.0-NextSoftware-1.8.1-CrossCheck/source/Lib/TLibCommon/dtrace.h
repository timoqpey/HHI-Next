/*!
***********************************************************************
*  \file
*      dtrace.h
*  \brief
*      implementation of trace messages support for debugging
*  \author
*      Tobias Mayer  <tobias.mayer@hhi-extern.fraunhofer.de>
*      Valeri George <valeri.george@hhi-extern.fraunhofer.de>
*  \copyright
*      2013 Tobias Mayer, Valeri George
***********************************************************************
*/
#ifndef _DTRACE_H_
#define _DTRACE_H_

#include <stdio.h>

#include <string>
#include <list>
#include <map>
#include <set>
#include <vector>
#include <cstdarg>
#include <stdint.h>

class CDTrace;

typedef std::string CType;


struct dtrace_channel
{
  int channel_number;
  std::string channel_name;
};

typedef std::vector< dtrace_channel > dtrace_channels_t;

class Condition
{
public:
    CType type;
    bool ( *eval )( int, int );
    int rval;

    Condition( CType t, bool ( *efunc )( int,int ), int refval )
    : type(t), eval(efunc), rval(refval)
    {}
};

class Channel
{
    typedef std::vector<Condition> Rule;
public:
    Channel() : rule_list(), _active(false), _counter(0) {}
    void update( std::map< CType, int > state );
    bool active() { return _active; }
    void add( Rule rule );
    void incrementCounter() { _counter++; }
    int64_t getCounter() { return _counter; }
private:
    std::list< Rule > rule_list;
    bool _active;
    int64_t _counter;
};

class CDTrace
{
  typedef std::pair< CType, int > state_type;
  //friend class Rules;
private:
    bool          copy;
    FILE         *m_trace_file;
    int           m_error_code;

    typedef std::string Key;
    typedef std::vector<std::string> vstring;
    typedef std::map< Key, int > channel_map_t;
    std::vector< Channel > chanRules;
    std::set< CType > condition_types;
    std::map< CType, int > state;
    std::map< Key, int > deserializationTable;

public:
    CDTrace() : copy(false), m_trace_file(NULL) {}
    CDTrace( const char *filename, vstring channel_names );
    CDTrace( const char *filename, const dtrace_channels_t& channels );
    CDTrace( const std::string& sTracingFile, const std::string& sTracingRule, const dtrace_channels_t& channels );
    CDTrace( const CDTrace& other );
    CDTrace& operator=( const CDTrace& other );
    ~CDTrace();
    void swap         ( CDTrace& other );
    int  addRule      ( std::string rulestring );
    void dtrace       ( int, const char *format, /*va_list args*/... );
    void dtrace_repeat( int, int i_times, const char *format, /*va_list args*/... );
    bool update       ( state_type stateval );
    int  init( vstring channel_names );
    int  getLastError() { return m_error_code;  }
    const char*  getChannelName( int channel_number );
    void getChannelsList( std::string& sChannels );
    std::string getErrMessage();
    int64_t getChannelCounter( int channel ) { return chanRules[channel].getCounter(); }
};

// Class CBitCounter: a simple straight forward counter for processed bits
typedef enum {
  BS_SIGNALING,
  BS_TEXTURE,
  BS_OTHER,
  BS_NUM_STAGES,
} BITCOUNTER_STAGE;

class CBitCounter
{
public:
  CBitCounter(){};
  ~CBitCounter(){};

  void reset()
  {
    for( int i = 0; i < BS_NUM_STAGES; i++ ) m_bits[i] = 0;
  }
  void startCount( BITCOUNTER_STAGE s, int bits )
  {
    m_stage = s;
    m_prevBits = bits;
  }
  void startCountWithReset( BITCOUNTER_STAGE s, int bits )
  {
    reset();
    startCount( s, bits );
  }
  void stopCount( int bits )
  {
    m_bits[m_stage] += bits - m_prevBits;
  }
  void accumBits( BITCOUNTER_STAGE s, int bits )
  {
    m_bits[m_stage] += bits - m_prevBits;
    startCount( s, bits );
  }
  void accumulate( CBitCounter* pcSrc )
  {
    for( int j = 0; j < BS_NUM_STAGES; j++ )
      m_bits[j] += pcSrc->m_bits[j];
  }

public:
  int64_t m_bits[BS_NUM_STAGES];
private:
  int m_prevBits;
  BITCOUNTER_STAGE m_stage;
};

#endif // _DTRACE_H_

