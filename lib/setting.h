#ifndef _SETTING_H_
#define _SETTING_H_

#include <string>
#include <fstream>
#include <map>

#include <iostream> //cout

namespace cpp_io {

class Setting {
	public:
		Setting(const std::string& file_name, std::ios_base::openmode mode = std::ios_base::in|std::ios_base::out);
		~Setting();

		template <class T>
		friend Setting& operator <<(Setting &setting, const std::map<std::string, T> &val_map);
		/// Start new group/ hierarchy
		void begin_group(const std::string& groupname);
		// End current group/ hierarchy
		void end_group();
		/// Set key-value-pair for current group
		template <class T>
		void set_value(const std::string &key, const T& value);
		template <class T>
		void value(const std::string &key, T& value);

	private:
		/// config file
		std::fstream conf_file;
		/// prepend string
		std::string pre_string;
		/// current group
		std::string cur_group;
	
		Setting(const Setting &);
		void operator=(const Setting &);

		std::streampos find_group(const std::string &group);
	
};


// ----------------------------------------------------------------------------
// Setting
// ----------------------------------------------------------------------------
template <class T>
inline void Setting::set_value(const std::string &key, const T& value) {
	//FIXME: look for existing key first
	conf_file << pre_string << key << " = " << value << std::endl;
	conf_file.flush();
}

template <class T>
inline void Setting::value(const std::string &key, T& value) {
	using namespace std;

	streampos spos = find_group(cur_group);
	if(spos == -1
	|| spos == ios::end) {//found no group
		//assume not explicit given core group
		//at beginning of conf_file
		conf_file.clear();
		conf_file.seekg(ios_base::beg);
	}

	string line;
	while(getline(conf_file, line)) { //read line by line
		if(line.empty()) continue;
		string::size_type start = line.find_first_not_of(" \t\n");
		//skip comment lines
		if(start != string::npos && line[start] == '#') 
			continue;

		//stop at next group
		if(line[start] == '[') break;

		//tokenize line
		string::size_type end = line.find_first_of(" \t=:", start);
		if(end != string::npos
		&& line.substr(start, end-start) == key) {
			//found key
			start = line.find_first_not_of(" \t=:", end);
			if(start == string::npos) continue;
			istringstream val_stream(line.substr(start, string::npos));
			val_stream >> value;
			break;	
		}
	}
}


} // namespace cpp_io

#endif
