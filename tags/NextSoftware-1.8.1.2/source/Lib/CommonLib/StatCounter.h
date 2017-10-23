/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef STATCOUNTER_H_
#define STATCOUNTER_H_


#include <string>
#include <ostream>
#include <vector>
#include <cstdarg>


////////////////////////////////////////////////////////////////////////////////////
// StatCounter

template<typename T>
class StatCounter
{
public:
  StatCounter() : m_counter( 0 ), m_counter_name(""), m_isPercentageOutput( false ){}
  StatCounter( T init_val ) : m_counter( init_val ), m_counter_name( "" ), m_isPercentageOutput( false ){}
  StatCounter( T init_val, const char* name ) : m_counter( init_val ), m_counter_name( name ), m_isPercentageOutput( false ){}
  ~StatCounter(){}

  StatCounter& operator+=( const StatCounter& other ) {
    m_counter += other.m_counter;
    return *this;
  }

  StatCounter& operator-=( const StatCounter& other ) {
    m_counter -= other.m_counter;
    return *this;
  }

  StatCounter& operator= ( const T& other )
  {
    m_counter = other;
    return *this;
  }

  bool operator<  ( const T& other ) { return    m_counter < other; }
  bool operator>  ( const T& other ) { return    m_counter > other; }
  bool operator>= ( const T& other ) { return !( m_counter < other ); }
  bool operator<= ( const T& other ) { return !( m_counter > other ); }

  StatCounter& operator++()
  {
    ++m_counter;
    return *this;
  }

  StatCounter& operator++(int)
  {
    m_counter++;
    return *this;
  }

  friend std::ostream& operator<<( std::ostream& os, const StatCounter<T>& cnt )
  {
    os << cnt.m_counter;
    return os;
  }

  friend std::istream& operator>>( std::istream& os, StatCounter<T>& cnt )
  {
    os >> cnt.m_counter;
    return os;
  }

  std::ostream& name( std::ostream& os )
  {
    os << m_counter;
    return os;
  }  
  
  void reset( T val )
  {
    m_counter = val;
  }

  void              setName( const char* name ) { m_counter_name = std::string( name ); }
  const std::string getName() const             { return m_counter_name; }
  T&                val()                       { return m_counter; }
  const T&          val() const                 { return m_counter; }
  Double            percentageFrom( const T& v ) const                         { return (Double)m_counter * 100.0 / (Double)v; }
  Double            percentageFrom( const StatCounter<T>& other ) const        { return (Double)m_counter * 100.0 / (Double)other.m_counter; }
  Double            diffAndPercentageFrom( const StatCounter<T>& other ) const { return (Double)(m_counter - other.m_counter) * 100.0 / (Double)other.m_counter; }
  void              setPercentageDependence( size_t idx )                      { m_dependenceIdx = idx, m_isPercentageOutput = true; }
  bool              isPercentageOutput() const { return m_isPercentageOutput; }
  size_t            getDependenceIdx()   const { return m_dependenceIdx;  }
private:
  T           m_counter;
  std::string m_counter_name;
  bool        m_isPercentageOutput;
  size_t      m_dependenceIdx;
};


////////////////////////////////////////////////////////////////////////////////////
// StatCounter1D

template<typename T>
class StatCounter1D
{
public:
  StatCounter1D(){ m_counters.resize( 1, 0 ); }
  StatCounter1D( size_t num_counters, T init_val = 0 ){ m_counters.resize( num_counters, init_val ); }
  StatCounter1D( size_t num_counters, T init_val, bool dummy ){ m_counters.resize( num_counters, init_val ); }
  StatCounter1D( size_t num_counters, T init_val, ... )
  {
    m_counters.resize( num_counters, init_val );
    {
      va_list args;
      va_start( args, init_val );
      for( size_t i = 0; i < num_counters; i++ )
      {
        const char* val = va_arg( args, const char* );
        if( val )
        {
            m_counters[i].setName( val );
        }
      }
      va_end( args );
    }
  }
  ~StatCounter1D(){}

  void resize( size_t num_counters, T init_val )
  {
      m_counters.resize( num_counters, init_val );
  }

  StatCounter1D& operator+=( const StatCounter1D& other ) {
    auto i1 = m_counters.begin();
    auto i2 = other.m_counters.cbegin();
    for( ; i1 != m_counters.end() && i2 != other.m_counters.cend(); ++i1, ++i2 ) {
      *i1 += *i2;
    }
    return *this;
  }

  StatCounter1D& operator-=( const StatCounter1D& other ) {
    auto i1 = m_counters.begin();
    auto i2 = other.m_counters.cbegin();
    for( ; i1 != m_counters.end() && i2 != other.m_counters.cend(); ++i1, ++i2 ) {
      *i1 -= *i2;
    }
    return *this;
  }

  StatCounter1D& operator= ( const T& other )
  {
    m_counters[0] = other;
    return *this;
  }

  StatCounter1D& operator= ( const StatCounter1D& other )
  {
    CHECK( m_counters.size() != other.m_counters.size(), "Accessing counters with differen number of elements!" );
    for( size_t i = 0; i < m_counters.size(); i++ ) m_counters[i] = other.m_counters[i];
    return *this;
  }

  StatCounter<T>&       operator[]( std::size_t idx )       { return m_counters[idx]; }
  const StatCounter<T>& operator[]( std::size_t idx ) const { return m_counters[idx]; }

  void inc( int cntIdx )
  {
    ++m_counters[cntIdx];
  }

  void add( int cntIdx, T val )
  {
    m_counters[cntIdx] += val;
  }

  void reset( T val )
  {
    for( size_t i = 0; i < m_counters.size(); i++ )
      m_counters[i].reset( val );
  }

  size_t size() const { return m_counters.size();  }

  T total( T accum = 0 )
  {
    for( size_t i = 0; i < m_counters.size(); i++ )
      accum += m_counters[i].val();
    //accum = std::accumulate( m_counters.begin(), m_counters.end() - 1, accum, std::plus<StatCounter<T>>() );
    return accum;
  }

  std::ostream& streamOutNames( std::ostream& os, size_t w ) const
  {
    for( size_t i = 0; i < m_counters.size(); i++ )
      os << std::setw( (m_counters[i].isPercentageOutput() ? w*2: w) ) << m_counters[i].getName();
    return os;
  }

  std::ostream& streamOutValueInPercentage( std::ostream& os, const StatCounter<T>& cnt, T dep, size_t w ) const
  {
      os << std::fixed << std::setw( w ) << std::setprecision( 1 ) << cnt.percentageFrom( dep ) << "%";
      return os;
  }

  std::ostream& streamOutValuesInPercentage( std::ostream& os, T dep, int w ) const
  {
      for( size_t i = 0; i < m_counters.size(); i++ )
      {
          os << std::setw( w ) << m_counters[i];
          streamOutValueInPercentage( os, m_counters[i], dep, w );
      }
      return os;
  }

  std::ostream& streamOutValues( std::ostream& os, size_t w ) const
  {
    for( size_t i = 0; i < m_counters.size(); i++ )
    {
        os << std::setw( w ) << m_counters[i];
        if( m_counters[i].isPercentageOutput() )
            streamOutValueInPercentage( os, m_counters[i], m_counters[m_counters[i].getDependenceIdx()].val(), w );
    }
    return os;
  }

  std::ostream& streamOutResults( std::ostream& os, size_t w, bool isHorizontal = true )
  {
      if( isHorizontal )
      {
          streamOutNames( os, w );
          streamOutValues( os, w );
      }
      else
      {
          size_t maxW = 0;
          for( size_t i = 0; i < m_counters.size(); i++ )
              if( m_counters[i].getName().length() > maxW )
                  maxW = m_counters[i].getName().size();

          for( size_t i = 0; i < m_counters.size(); i++ )
          {
              os << std::setw( maxW > w ? maxW: w ) << m_counters[i].getName() << ": ";
              os << std::setw( w ) << m_counters[i];
              if( m_counters[i].isPercentageOutput() )
                  streamOutValueInPercentage( os, m_counters[i], m_counters[m_counters[i].getDependenceIdx()].val(), w );
              os << std::endl;
          }
      }
      return os;
  }

  std::ostream& streamOutValuesInPercentage( std::ostream& os )
  {
      T totalSum = total();
      for( size_t i = 0; i < m_counters.size(); i++ )
          os << std::fixed << std::setw( 6 ) << std::setprecision( 1 ) << m_counters[i].percentageFrom( totalSum );
      return os;
  }

  inline bool isEmpty() { for( size_t i = 0; i < m_counters.size(); i++ ){ if( m_counters[i].val() ) return false; } return true; }


  //template<typename T>
  friend std::ostream& operator<<( std::ostream& os, const StatCounter1D& cnt )
  {
    if( !cnt.m_counters.empty() )
    {
      size_t i = 0;
      for( ; i < cnt.m_counters.size() - 1; i++ )
        os << cnt.m_counters[i] << " ";

      os << cnt.m_counters[i];
    }
    return os;
  }

  friend std::istream& operator>>( std::istream& os, StatCounter1D& cnt )
  {
    if( !cnt.m_counters.empty() )
    {
      size_t i = 0;
      for( ; i < cnt.m_counters.size() - 1; i++ )
        os >> cnt.m_counters[i];

      os >> cnt.m_counters[i];
    }
    return os;
  }

private:
  std::vector<StatCounter<T>> m_counters;
};

template<typename T>
class StatCounter2D
{
public:
    StatCounter2D(){ m_counters.resize( 1, 0 ); }
    StatCounter2D( size_t yDim, size_t xDim, T init_val = 0 )
    {
        m_counters.resize( yDim );
        for( size_t y = 0; y < yDim; y++ )
            m_counters[y].resize( xDim, 0 );
    }

    StatCounter1D<T>&       operator[]( size_t y )       { return m_counters[y]; }
    const StatCounter1D<T>& operator[]( size_t y ) const { return m_counters[y]; }

    StatCounter2D& operator+=( const StatCounter2D& other ) {
        auto i1 = m_counters.begin();
        auto i2 = other.m_counters.cbegin();
        for( ; i1 != m_counters.end() && i2 != other.m_counters.cend(); ++i1, ++i2 ) {
            *i1 += *i2;
        }
        return *this;
    }

    T total( T accum = 0 )
    {
        for( size_t j = 0; j < m_counters.size(); j++ )
            m_counters[j].total( accum );
        return accum;
    }

    void reset( T val )
    {
        for( size_t i = 0; i < m_counters.size(); i++ )
            m_counters[i].reset( val );
    }
private:
    std::vector<StatCounter1D<T>> m_counters;
};
#endif /* STATCOUNTER_H_ */
