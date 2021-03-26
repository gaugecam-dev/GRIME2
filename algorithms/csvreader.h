#ifndef CSVREADER_H
#define CSVREADER_H

#include <fstream>
#include <vector>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Example usage
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// int main()
// {
//     // Creating an object of CSVWriter
//     CSVReader reader("example.csv");

//     // Get the data from CSV File
//     std::vector<std::vector<std::string> > dataList = reader.getData();

//     // Print the content of row by row on screen
//     for(std::vector<std::string> vec : dataList)
//     {
//         for(std::string data : vec)
//         {
//             std::cout<<data << " , ";
//         }
//         std::cout<<std::endl;
//     }
//     return 0;
// }
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

namespace gc
{

/*
 * A class to read data from a csv file.
 */
class CSVReader
{
    std::string fileName;
    std::string delimeter;

public:
    CSVReader( std::string filename, std::string delm = "," ) :
            fileName( filename),
            delimeter( delm )
    { }

    // Function to fetch data from a CSV File
    std::vector< std::vector< std::string > > getData()
    {
        std::ifstream file( fileName );

        std::vector< std::vector< std::string > > dataList;

        std::string line = "";
        // Iterate through each line and split the content using delimeter
        while ( getline( file, line ) )
        {
            if ( !line.empty() )
            {
                if ( '#' != line[ 0 ] )
                {
                    std::vector< std::string > vec;
                    boost::algorithm::split( vec, line, boost::is_any_of( delimeter ) );
                    dataList.push_back( vec );
                }
            }
        }
        // Close the File
        file.close();

        return dataList;
    }
};

} // namespace gc

#endif // CSVREADER_H
