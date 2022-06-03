//--------------------------------------------------
// Implementation of class ArgUtils
//
// @author: Wild Boar
//
// @date: 2022-03-11
//--------------------------------------------------

#include "ArgUtils.h"
using namespace NVL_App;

//--------------------------------------------------
// Load Parameters
//--------------------------------------------------

/**
 * @brief Load parameters
 * @param appName The name of the application
 * @param argc The number of incomming parameter arguments
 * @param argv The incomming variables
 * @return NVLib::Parameters * Returns a NVLib::Parameters *
 */
NVLib::Parameters * ArgUtils::Load(const string& appName, int argc, char ** argv)
{
	if (argc > 2) throw runtime_error(NVLib::Formatter() << "Usage: " << appName << " <config.xml>");
	auto configPath = argc == 1 ? string("config.xml") : string(argv[1]);
	return NVLib::ParameterLoader::Load(configPath);
}

//--------------------------------------------------
// Extract Parameter Values
//--------------------------------------------------

/**
 * @brief Retrieve the given string value
 * @param parameters The parameters that we are extracting from
 * @param key The key value that we are extracting
 * @return string The string value
 */
string ArgUtils::GetString(NVLib::Parameters * parameters, const string& key) 
{
	if (!parameters->Contains(key)) throw runtime_error("The parameters does not contain the required value: " + key);
	auto value = parameters->Get(key);
	return value;
}

/**
 * @brief Retrieve the given integer value 
 * @param parameters The parameters that we are extracting from
 * @param key The key value that we are extracting 
 * @return int The integer value that we are extracting
 */
int ArgUtils::GetInteger(NVLib::Parameters * parameters, const string& key) 
{
	if (!parameters->Contains(key)) throw runtime_error("The parameters does not contain the required value: " + key);
	auto value = parameters->Get(key);
	return NVLib::StringUtils::String2Int(value);
}

/**
 * @brief Retrieve the given double value
 * @param parameters The parameters that we are extracting from
 * @param key The key value that we are extracting 
 * @return double The double value that is being extracted
 */
double ArgUtils::GetDouble(NVLib::Parameters * parameters, const string& key) 
{
	if (!parameters->Contains(key)) throw runtime_error("The parameters does not contain the required value: " + key);
	auto value = parameters->Get(key);
	return NVLib::StringUtils::String2Double(value);
}

/**
 * @brief Retrieve the boolean value that is being extracted
 * @param parameters The parameters that we are extracting from
 * @param key The given key value
 * @return true If the value is true
 * @return false If the value is false
 */
bool ArgUtils::GetBoolean(NVLib::Parameters * parameters, const string& key) 
{
	if (!parameters->Contains(key)) throw runtime_error("The parameters does not contain the required value: " + key);
	auto value = parameters->Get(key);
	return NVLib::StringUtils::String2Bool(value);
}
