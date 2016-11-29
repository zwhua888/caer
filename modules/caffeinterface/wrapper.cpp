/* C wrapper to caffe interface
 *  Author: federico.corradi@inilabs.com
 */
#include "classify.hpp"
#include "wrapper.h"

extern "C" {

MyClass* newMyClass() {
	return new MyClass();
}

int MyClass_file_set(MyClass* v, int * picture, int size){
	v->file_set(picture, size);
}

char * MyClass_file_get(MyClass* v) {
	return v->file_get();
}

void MyClass_init_network(MyClass *v) {
	return v->init_network();
}

void deleteMyClass(MyClass* v) {
	delete v;
}

}
