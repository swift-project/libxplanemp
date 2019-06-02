#ifndef STRING_UTILS_H
#define STRING_UTILS_H

#include <string>
#include <vector>
#include <algorithm>
#include <fstream>
#include <cctype>

inline std::string getFileContent(const std::string &filename)
{
	std::ifstream in(filename, std::ifstream::in | std::ifstream::binary);
	std::string content;
    in.seekg(0, std::ios::end);
    const auto size = static_cast<std::string::size_type>(in.tellg());
	content.reserve(size);
	in.seekg(0, std::ios::beg);
	content.assign((std::istreambuf_iterator<char>(in)),
		std::istreambuf_iterator<char>());
	std::string result;
	result.reserve(size);
	for (decltype(content)::size_type begin = 0; begin != content.npos; )
	{
		auto end = content.find_first_of("\r\n", begin, 2);
		if (end == content.npos) { end = content.size(); }
		result.append(content, begin, end - begin);
		result.push_back('\n');
		begin = content.find_first_not_of("\r\n", end, 2);
	}
	return result;
}

inline void writeFileContent(const std::string &filename, const std::string &content)
{
	std::ofstream out(filename, std::ios::out | std::ios::binary);
	out << content;
}

inline bool fileExists(const std::string &filename)
{
	return std::filebuf().open(filename, std::ifstream::in | std::ifstream::binary);
}

// trim from start (in place)
static inline void ltrim(std::string &s)
{
	s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](char c) { return !std::isspace(c); }));
}

// trim from end (in place)
static inline void rtrim(std::string &s)
{
	s.erase(std::find_if(s.rbegin(), s.rend(), [](char c) { return !std::isspace(c); }).base(), s.end());
}

// trim from both ends (in place)
static inline void trim(std::string &s)
{
	ltrim(s);
	rtrim(s);
}

inline std::vector<std::string> tokenize(const std::string &str, const std::string &delim = " \t\f\v\r\n")
{
	std::string dup = str;
	std::vector<std::string> result;
	if (delim.empty())
	{
		result.push_back(dup);
		return result;
	}

	if (dup.empty()) return result;

	while (true)
	{
		auto position = dup.find_first_of(delim);
		std::string token = dup.substr(0, position);

		if (!token.empty())
		{
			result.push_back(token);
		}

		// Nothing remaining
		if (position == std::string::npos) return result;

		dup = dup.substr(position + 1);
	}
}


#endif // STRING_UTILS_H

