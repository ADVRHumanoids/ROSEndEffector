/*
 * Copyright (C) 2019 IIT-HHCM
 * Author: Arturo Laurenzi
 * email:  arturo.laurenzi@iit.it
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#ifndef __ROSEE_UTILS__
#define __ROSEE_UTILS__

#include <cmath>
#include <memory>

//to find relative path for the config files and create directories
#include <boost/filesystem.hpp>
#include <fstream>

namespace ROSEE
{

namespace Utils
{
    
static bool create_directory(std::string pathDirectory){
    boost::filesystem::path path(pathDirectory);
    return boost::filesystem::create_directories(path);
}

static void out2file ( std::string pathFile, std::string output) {
    std::ofstream fout ( pathFile );
    fout << output;
}
    
static inline int binomial_coefficent(int n, int k) {

    if (k == 0 || k == n){
        return 1;
    }
    return Utils::binomial_coefficent(n - 1, k - 1) + Utils::binomial_coefficent(n - 1, k);
    
}
    
    
static std::string getPackagePath() {
    
    boost::filesystem::path path(__FILE__);
    path.remove_filename();
    return path.string() + "/../../";
}

template <class T>
static std::vector<std::string> extract_keys(std::map<std::string, T> const& input_map) {
  std::vector<std::string> retval;
  for (auto const& element : input_map) {
    retval.push_back(element.first);
  }
  return retval;
}


template <typename SignalType>
class SecondOrderFilter
{

public:

    typedef std::shared_ptr<SecondOrderFilter<SignalType>> Ptr;

    SecondOrderFilter() :
        _omega ( 1.0 ),
        _eps ( 0.8 ),
        _ts ( 0.01 ),
        _reset_has_been_called ( false )
    {
        computeCoeff();
    }

    SecondOrderFilter ( double omega, double eps, double ts, const SignalType& initial_state ) :
        _omega ( omega ),
        _eps ( eps ),
        _ts ( ts ),
        _reset_has_been_called ( false )
    {
        computeCoeff();
        reset ( initial_state );
    }

    void reset ( const SignalType& initial_state )
    {
        _reset_has_been_called = true;
        _u = initial_state;
        _y = initial_state;
        _yd = initial_state;
        _ydd = initial_state;
        _udd = initial_state;
        _ud = initial_state;
    }

    const SignalType& process ( const SignalType& input )
    {

        if ( !_reset_has_been_called ) {
            reset ( input*0 );
        }


        _ydd = _yd;
        _yd = _y;
        _udd = _ud;
        _ud = _u;


        _u = input;
        _y = 1.0/_a0 * ( _u + _b1*_ud + _b2*_udd - _a1*_yd - _a2*_ydd );

        return _y;
    }

    const SignalType& getOutput() const
    {
        return _y;
    }

    void setOmega ( double omega )
    {
        _omega = omega;
        computeCoeff();
    }

    double getOmega()
    {
        return _omega;
    }

    void setDamping ( double eps )
    {
        _eps = eps;
        computeCoeff();
    }

    double getDamping()
    {
        return _eps;
    }

    void setTimeStep ( double ts )
    {
        _ts = ts;
        computeCoeff();
    }

    double getTimeStep()
    {
        return _ts;
    }

private:

    void computeCoeff()
    {
        _b1 = 2.0;
        _b2 = 1.0;

        _a0 = 1.0 + 4.0*_eps/ ( _omega*_ts ) + 4.0/std::pow ( _omega*_ts, 2.0 );
        _a1 = 2 - 8.0/std::pow ( _omega*_ts, 2.0 );
        _a2 = 1.0 + 4.0/std::pow ( _omega*_ts, 2.0 ) - 4.0*_eps/ ( _omega*_ts );

    }

    double _omega;
    double _eps;
    double _ts;

    double _b1, _b2;
    double _a0, _a1, _a2;

    bool _reset_has_been_called;

    SignalType _y, _yd, _ydd, _u, _ud, _udd;

};


}

}

#endif // __ROSEE_UTILS__
