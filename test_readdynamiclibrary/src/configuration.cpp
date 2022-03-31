//
// Created by quan on 2022/3/24.
//

#include <iostream>

#include "test_readdynamiclibrary/configuration.h"
#include "Python.h"

ParameterParser::ParameterParser()
{
    std::string shared_name = "test_toml";
    ReadParmtersFromSharedLibrary(shared_name);
}

bool ParameterParser::LoadDefaultParameters(const std::string & toml_dynamic_library_name)
{

}

bool ParameterParser::ParameterParser::HasKey(const std::string& key)
{

}

std::string ParameterParser::GetString(const std::string& key)
{

}

double ParameterParser::GetDouble(const std::string& key)
{

}

int ParameterParser::GetInt(const std::string& key)
{

}

bool ParameterParser::GetBool(const std::string& key)
{

}

bool ParameterParser::SetString(const std::string& key, const std::string & value)
{

}

bool ParameterParser::SetDouble(const std::string& key, const double & value)
{

}

bool ParameterParser::SetInt(const std::string& key, const int & value)
{

}

bool ParameterParser::SetBool(const std::string& key, const bool & value)
{

}

bool ParameterParser::ReadParmtersFromSharedLibrary(
    const std::string & shared_name)
{
    std::string import = "import " + shared_name + "\n";
    Py_Initialize();
    if (!Py_IsInitialized()) {
        return false;
    }

    PyRun_SimpleString("import sys; import toml;");
    PyRun_SimpleString("sys.path.append('./')");
    PyRun_SimpleString(import.c_str());

    PyObject* pName  = PyUnicode_FromString(shared_name.c_str());
    PyObject* pModule = PyImport_Import(pName);
    if( pModule == nullptr){
        std::cout <<"module not found" << std::endl;
        return false;
    }

    // get function
    std::string python_function = "get_" + shared_name + "_data";
    PyObject* pFunc = PyObject_GetAttrString(pModule, python_function.c_str());
    if( !pFunc || !PyCallable_Check(pFunc)) {
        std::cout << "Not found function : " << python_function << std::endl;
        return false;
    }

    // call function
    PyObject* function_ret = PyObject_CallObject(pFunc, NULL);
    if (!function_ret) {
        std::cout << "Call function error.!" << std::endl;
    }

    // istream parse toml
    char* result;
    PyArg_Parse(function_ret, "s", &result);//转换返回类型
    std::string toml_data = std::string(result);
    std::istringstream iss(toml_data);
    file_solver_handler_ = toml::parse(iss );

    std::string ab;
    auto name = toml::find<std::string>(file_solver_handler_, "A", "f");
    std::cout << "name = " << name << std::endl;

    Py_DECREF(pName);
    Py_DECREF(pModule);
    Py_DECREF(function_ret);
    Py_Finalize();

    return true;
}


