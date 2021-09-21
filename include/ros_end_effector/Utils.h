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
#include <iostream>
#include <dlfcn.h>

//to find relative path for the config files and create directories
#include <boost/filesystem.hpp>
#include <fstream>

#include <chrono>
#include <atomic>

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

// static std::string get_environment_variable( std::string const & key )
// {
//     char * val = getenv( key.c_str() );
//     return val == NULL ? std::string("") : std::string(val);
// }

static std::vector <std::string> getFilesInDir ( std::string pathFolder ) {
    
    boost::filesystem::path p (pathFolder);
    std::vector <std::string> retVect;
    
    if (! boost::filesystem::exists(p) ) {
        std::cerr << "[ERROR " << __func__ << "] path '" << pathFolder << "' does not exists" << std::endl;
        return retVect;
    }
    
    if (! boost::filesystem::is_directory(p)){ 
        std::cerr << "[ERROR " << __func__ << "] path '" << pathFolder << "' is not a directory" << std::endl;
        return retVect;
    }
    
    for (boost::filesystem::directory_entry& x : boost::filesystem::directory_iterator(p)) {
        retVect.push_back (x.path().filename().string() );
    }
    
    return retVect;
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

template <class KeyType, class ValueType>
static std::vector<KeyType> extract_keys(std::map<KeyType, ValueType> const& input_map) {
  std::vector<KeyType> retval;
  for (auto const& element : input_map) {
    retval.push_back(element.first);
  }
  return retval;
}

/** 
 * @brief Extract all the string in the set keys of a map. All string are put togheter so
 * the original meaning of each set is lost
 * @param input_map the map where extract the keys
 * @param max_string_number the max number of different string among all the set keys. 
 *      Useful to not iterate all the map if not necessary. With default value = 0 all 
 *      map is iterated.
 * @return vector of extracted string of set keys (string in this vect will be unique)
 */
template <class T>
static std::vector<std::string> extract_keys_merged(
    std::map<std::set<std::string>, T> const& input_map, unsigned int max_string_number = 0) {
    
    std::set<std::string> allStrings;
    // if else so we do not check in the for the max_string_number if it is not used (ie ==0)

    if (max_string_number == 0) {
        for (auto const& element : input_map) {
            allStrings.insert( element.first.begin(), element.first.end() );
        }
            
    } else {
        for (auto const& element : input_map) {
            allStrings.insert(element.first.begin(), element.first.end());
            if (max_string_number == allStrings.size()){
                break;
            }
            if (max_string_number < allStrings.size() ) {
                std::cerr << "[ERROR]" << __func__ << " You passed " << max_string_number
                << " but I found more unique strings in the set keys ( " << allStrings.size()
                << " found)" << std::endl;
                return std::vector<std::string>();
            }
        }
    }
    std::vector<std::string> retval (allStrings.begin(), allStrings.end());
    return retval;
}

/**
 * @brief See above, this is the version with pair instead of set
 */
template <class T>
static std::vector<std::string> extract_keys_merged(
    std::map<std::pair<std::string,std::string>, T> const& input_map, unsigned int max_string_number = 0) {
    
    std::set<std::string> allStrings;
    // if else so we do not check in the for the max_string_number if it is not used (ie ==0)

    if (max_string_number == 0) {
        for (auto const& element : input_map) {
            allStrings.insert( element.first.first);
            allStrings.insert( element.first.second);
        }
            
    } else {
        for (auto const& element : input_map) {
            allStrings.insert( element.first.first);
            allStrings.insert( element.first.second);
            if (max_string_number == allStrings.size()){
                break;
            }
            if (max_string_number < allStrings.size() ) {
                std::cerr << "[ERROR]" << __func__ << " You passed " << max_string_number
                << " but I found more unique strings in the pair keys ( " << allStrings.size()
                << " found)" << std::endl;
                return std::vector<std::string>();
            }
        }
    }
    std::vector<std::string> retval (allStrings.begin(), allStrings.end());
    return retval;
}

/** @brief Return false if two maps have different keys. 
 * The type of the keys (@p typename) must be the same obviously,
 * but the values (@p valueType1 and @p valueType2) can be anything, because they are not considered
*/
template <typename keyType, typename valueType1, typename valueType2>
bool keys_equal (std::map <keyType, valueType1> const &lhs, std::map<keyType, valueType2> const &rhs) {

    auto pred = [] (decltype(*lhs.begin()) a, decltype(*rhs.begin()) b)
                   { return (a.first == b.first); };


    return lhs.size() == rhs.size()
        && std::equal(lhs.begin(), lhs.end(), rhs.begin(), pred);
}

template <typename Map1, typename Map2>
struct DifferentKeysException : public std::exception {
    const Map1 *map1;
    const Map2 *map2;
    
    DifferentKeysException(const Map1 *map1, const Map2 *map2) :
        map1(map1), map2(map2) {}
        
    const char * what () const throw () {
        std::stringstream output;
        output << "First map keys:\n";
        for (auto it : *map1) {
            output << "\t" << it.first << "\n";
        }
        output << ("Second map keys:\n");
        for (auto it : *map2) {
            output << "\t" << it.first << "\n";
        }
        std::cerr << output.str().c_str() << std::endl;

        return "Maps have different keys";
    }
};

/**
 * @brief Utils to dynamically load an object. This is used to dynamically load 
 *  a derived object from a node that only knows the base interface. 
 *  For example, we call the create_object(ros::nodeHandle) method of a derived EEHAL class
 *  The object must be a library which will return a RetType pointer with the \p function_name
 *  This function will "convert" to smart pointer for convenience
 * @param lib_name the name of the compiled library (eg DummyHal). Do not add the suffix .so
 * @param function_name The method of \param lib_name which will return a RetType*. 
 * @param args arguments for the \p function_name , if the case
 * @return std::shared_ptr<RetType> a pointer to the new created object
 */
template <typename RetType, typename... Args>
std::unique_ptr<RetType> loadObject(std::string lib_name, 
                                    std::string function_name,
                                    Args... args) {
    
    if (lib_name.empty()) {
        
        std::cerr << "[Utils::loadObject] ERROR: Please specify lib_name" << std::endl;
        return nullptr;
    } 
    
    std::string lib_name_path = "lib" + lib_name +".so"; 

    //clear old errors
    dlerror();

    void* lib_handle = dlopen(lib_name_path.c_str(), RTLD_LAZY);
    auto error = dlerror();

    if (!lib_handle || error != NULL) {
        std::cerr << "[Utils::loadObject] ERROR in opening the library: " << error << std::endl;
        return nullptr;
    }
    
    //clear old errors
    dlerror();
    
    RetType* (*function)(Args... args);
    function = reinterpret_cast<RetType* (*)(Args... args)>(dlsym(lib_handle, function_name.c_str()));
    error = dlerror();
    if ( error != NULL)  {
        std::cerr << "[Utils::loadObject] ERROR in returning the function: " << error << std::endl;
        return nullptr;
    }
    
    RetType* objectRaw = function(args...);
    
    std::unique_ptr<RetType> objectPtr(objectRaw);
    
    dlclose(lib_handle);
    
    return objectPtr;
}

//default template as high_resolution_clock
//copied from https://codereview.stackexchange.com/questions/196245/extremely-simple-timer-class-in-c
template <typename Clock = std::chrono::high_resolution_clock>
class Timer
{
    typename Clock::time_point start_point;

public:
    Timer() : start_point(Clock::now()) {}
    
    void reset() { start_point = Clock::now(); }
    
    template <typename Rep = typename Clock::duration::rep, typename Units = typename Clock::duration>
    Rep elapsed_time() const
    {
        std::atomic_thread_fence(std::memory_order_relaxed);
        auto counted_time = std::chrono::duration_cast<Units>(Clock::now() - start_point).count();
        std::atomic_thread_fence(std::memory_order_relaxed);
        return static_cast<Rep>(counted_time);
    }
};

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
