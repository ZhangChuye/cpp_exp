#include<iostream>
#include <fstream>
#include <string>
#include <array>
#include <vector>

using namespace std;

void printVector(vector<float> &v){
    for (vector<float>::iterator it = v.begin(); it != v.end(); it ++)
    {
        std::cout << *it << std::endl;
    }
    
}

int main () 
{
    std::ifstream myfile ("/home/tingxfan/rerun-4DMOS/seq11.txt"); // this is equivalent to the above method

    vector<float> data;
    std::string mystring;

    if ( myfile.is_open() ) { // always check whether the file is open
        while (myfile)
        {   
            float tem;

            myfile >> tem;
            data.push_back(tem);

            // std::cout << tem << std::endl; // pipe stream's content to standard output
        }
    }

    std::cout << "Done" << std::endl;
    std::cout << "data size: " << data.size() << std::endl;
}
