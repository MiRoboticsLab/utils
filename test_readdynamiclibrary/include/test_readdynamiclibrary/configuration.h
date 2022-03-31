//
// Created by quan on 2022/3/24.
//

#ifndef CPPDEMOS_CONFIGURATION_H
#define CPPDEMOS_CONFIGURATION_H

#include "toml.hpp"


class ParameterParser
{
public:
    using ParameterHandler = toml::value;

    ParameterParser(const std::string & toml_filename);
    ParameterParser();
    ~ParameterParser() {}

    ParameterParser(const ParameterParser &) = delete;
    ParameterParser& operator=(const ParameterParser &) = delete;

    // Load toml configuration from dynamic library
    bool LoadDefaultParameters(const std::string & toml_dynamic_library_name);

    // Returns true if the key is in this config filename.
    bool HasKey(const std::string& key);

    // These methods CHECK() that the 'key' exists.
    std::string GetString(const std::string& key);
    double GetDouble(const std::string& key);
    int GetInt(const std::string& key);
    bool GetBool(const std::string& key);

    // Set key
    bool SetString(const std::string& key, const std::string & value);
    bool SetDouble(const std::string& key, const double & value);
    bool SetInt(const std::string& key, const int & value);
    bool SetBool(const std::string& key, const bool & value);

    // In order to handle the saving of toml files
    ParameterHandler& GetParameterHandler() { return file_solver_handler_; }

private:
    bool ReadParmtersFromSharedLibrary(const std::string & shared_name);
    toml::value file_solver_handler_;

};  // class ParameterParser

#endif //CPPDEMOS_CONFIGURATION_H