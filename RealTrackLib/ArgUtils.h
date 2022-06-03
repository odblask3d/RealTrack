//--------------------------------------------------
// Utilities: A set of utilities for the processing of incomming arguments
//
// @author: Wild Boar
//
// @date: 2022-03-11
//--------------------------------------------------

#pragma once

#include <iostream>
using namespace std;

#include <NVLib/Formatter.h>
#include <NVLib/StringUtils.h>
#include <NVLib/Parameters/ParameterLoader.h>

namespace NVL_App
{
	class ArgUtils
	{
	public:
		static NVLib::Parameters * Load(const string& appName, int argc, char ** argv);

		static string GetString(NVLib::Parameters * parameters, const string& key);
		static int GetInteger(NVLib::Parameters * parameters, const string& key);
		static double GetDouble(NVLib::Parameters * parameters, const string& key);
		static bool GetBoolean(NVLib::Parameters * parameters, const string& key);
	};
}
