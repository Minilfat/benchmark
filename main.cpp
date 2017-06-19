#include <iostream>
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <chrono>
#include <iomanip>


#include "bdm/point.h"
#include "bdm/octree_node.h"
#include "unibn/octree.h"



std::vector<bdm::Point> points;

template<typename T> void print(T t, const int& width)
{
    std::cout << std::left << std::setw(width) << t;
}



void readPoints(const std::string& filename)
{
    std::ifstream in(filename.c_str());
    if (!in) {
        std::cout << "Can't open the file!\n";
        exit(1);
    }
    std::string line;
    while (!in.eof())
    {
        std::getline(in, line);
        in.peek();

        typedef boost::tokenizer< boost::escaped_list_separator<char> , std::string::const_iterator, std::string> Tokenizer;
        boost::escaped_list_separator<char> separator('\\', ',', '\"');

        Tokenizer tok(line, separator);
        int j = 0;
        double coords[3];
        for (auto i : tok)
            coords[j++] = boost::lexical_cast<double>(i);

        points.push_back(bdm::Point(coords[0], coords[1], coords[2]));
    }

    in.close();
}

void initTime(bdm::OctreeNode<int>*& bdmTree, unibn::Octree<bdm::Point>& unibnTree, int bucketSize, double res[2])
{
    // all points range within (-50,50) interval
    // so main bound is the following
    bdmTree = new bdm::OctreeNode<int>(bdm::Bound(-50,-50,-50,50,50,50),100, bucketSize);

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < points.size(); i++) {
        bdmTree->Put(points[i], i);
    }
    auto end = std::chrono::high_resolution_clock::now();
    res[0] = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    start = std::chrono::high_resolution_clock::now();
    unibnTree.initialize(points, unibn::OctreeParams(bucketSize, true));
    end = std::chrono::high_resolution_clock::now();
    res[1] = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

};

void searchTime(bdm::OctreeNode<int>* bdmTree, unibn::Octree<bdm::Point>& unibnTree, double radius, double res[2])
{

    auto start = std::chrono::high_resolution_clock::now();
    auto a0 = bdmTree->GetNeighbors(radius);
    auto end = std::chrono::high_resolution_clock::now();
    res[0] = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    std::vector<uint32_t> a1;
    start = std::chrono::high_resolution_clock::now();
    for (uint32_t i = 0; i < points.size(); ++i)
    {
        unibnTree.radiusNeighbors<unibn::L2Distance<bdm::Point> >(points[i], radius, a1);
    }
    end = std::chrono::high_resolution_clock::now();
    res[1] = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    a0.clear();
    a1.clear();
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Filename of point cloud missing!\n";
        return -1;
    }
    std::string filename = argv[1];
    readPoints(filename);


    bdm::OctreeNode<int>* bdmTree;
    unibn::Octree<bdm::Point> unibnTree;
    double tempRes[2];

    std::cout << "Cloud contains " << points.size() << " points\n";
    std::cout << "\n";


    for (int i=64; i <=1024; i*=4) {
        std::cout << "***********************************\n";
        initTime(bdmTree, unibnTree, i, tempRes);
        std::cout << "Init time (ms)\n";
        print("Bucket size", 13); print("Bdm tree", 12); print("Unibn tree", 12); std::cout << std::endl;
        print(i, 13); print(tempRes[0], 12); print(tempRes[1], 12); std::cout << std::endl;
        std::cout << "-----------------------------------\n";

        std::cout << "Search time (ms)\n";
        double radius[4] = {0.01, 0.1, 0.5, 1.0} ;
        print("Radius", 13); print("Bdm", 12); print("Unibn", 12); std::cout<<std::endl;
        for (int k=0; k<4; k++) {
            searchTime(bdmTree, unibnTree, radius[k], tempRes);
            print(radius[k],13);print(tempRes[0],12); print(tempRes[1],12); std::cout<<std::endl;
        }

        delete bdmTree;
        unibnTree.clear();
        std::cout << "***********************************\n";
        std::cout << "\n";
    }

    return 0;
}
