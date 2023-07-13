


#include <iostream>
#include <fstream>

//using namespace std;

int main(){

  int a = 12;
  std::ofstream file;
  file.open("test_file.txt");
  file << a <<std::endl;
  file << "Help\n";
  file.close();
}
