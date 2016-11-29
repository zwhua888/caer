/* C wrapper to caffe interface
 *  Author: federico.corradi@gmail.com
 */
#ifndef __WRAPPER_H
#define __WRAPPER_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct MyClass MyClass;

MyClass* newMyClass();

int MyClass_file_set(MyClass* v, int * picture, int size);

char * MyClass_file_get(MyClass* v);

void MyClass_init_network(MyClass *v);

void deleteMyClass(MyClass* v);

#ifdef __cplusplus
}
#endif
#endif
