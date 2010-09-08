#include "setting.h"

namespace cpp_io {

using namespace std;

Setting::Setting(const std::string& file_name, std::ios_base::openmode mode) : 
	conf_file(file_name.c_str(), mode) {

	if (!conf_file) {
		//FIXME: throw exception
	}

	//"no-group" is alway at beginning of file
	group_pos = ios_base::beg;
}

Setting::~Setting() {
}

template <class T>
Setting::Setting& operator <<(Setting &setting, const std::map<std::string, T> &val_map) {

	typename map<std::string, T>::const_iterator i;
	for(i=val_map.begin(); i!=val_map.end(); ++i) {
		set_value(i->first, i->second);
	}

	return setting;
}

void Setting::begin_group(const std::string& groupname) {
	cur_group = groupname;

	group_pos = find_group(cur_group);
	if(group_pos == -1) { //found no group
		//fast-forward to end
		conf_file.clear();
		conf_file.seekp(ios_base::end);
		//insert group
		conf_file << "[" << cur_group << "]" << endl;
		group_pos = conf_file.tellp();
	}
	pre_string = "\t"; 
}

void Setting::end_group() {
	cur_group.clear();
	pre_string.clear();

	//rewind to "no-group"
	conf_file.clear();
	conf_file.seekg(ios_base::beg);
	group_pos = conf_file.tellg();
}

std::streampos Setting::find_group(const std::string &group) {
	//rewind to beginning
	conf_file.clear();
	conf_file.seekg(ios_base::beg);

	//"no-group" is always at beginning of file
	if(group.empty()) return conf_file.tellg();

	string line;
	while(getline(conf_file, line)) { //read line by line
		if(line.empty()) continue;
		string::size_type start = line.find_first_not_of(" \t\n");
		//skip every line not starting with [
		if(start != string::npos && line[start] != '[') 
			continue;
		//ignore whitespaces around group
		start = line.find_first_not_of("[ \t", start);
		if(start == string::npos) continue;
		string::size_type end = line.find_first_of("] \t", start);
		if(end == string::npos) continue;

		if(line.substr(start, end-start) == group)
			break; //found group
	}

	return conf_file.tellg();
}

} //namespace cpp_io
