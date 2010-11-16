#include "setting.h"

namespace cpp_io {

using namespace std;

Setting::Setting(const std::string& filename, std::ios_base::openmode mode) throw(const std::invalid_argument&) : 
		FStream(filename, mode) {

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

int Setting::begin_group(const std::string& groupname) {
	int rc = 0;

	std::streampos g_pos = find_group(groupname);
	if(g_pos == -1) { //found no group
		if(omode() & std::ios_base::out) {	//add new group
			cur_group = groupname;
			//fast-forward to end
			clear();
			seekp(ios_base::end);
			//insert group
			(*this) << "[" << cur_group << "]" << endl;
			group_pos = tellp();
			rc = 1;
		} else {
			rc = -1;
		}
	} else { //found group
		cur_group = groupname;
		group_pos =g_pos;
	}
	pre_string = "\t";

	return rc;
}

void Setting::end_group() {
	cur_group.clear();
	pre_string.clear();

	//rewind to "no-group"
	clear();
	seekg(ios_base::beg);
	group_pos = tellg();
}

std::streampos Setting::find_group(const std::string &group) {
	//rewind to beginning
	clear();
	seekg(ios_base::beg);

	//"no-group" is always at beginning of file
	if(group.empty()) return tellg();

	string line;
	while(std::getline(*this, line)) { //read line by line
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

	return tellg();
}

} //namespace cpp_io
