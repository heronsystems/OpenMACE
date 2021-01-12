#ifndef STRING_METHODS_H
#define STRING_METHODS_H

#include <string>
#include <iostream>
#include <vector>

class StringMethods
{
public:
    // ******************* Helper functions for parsing: *******************
    static std::vector<std::string> SplitString(const std::string &str, const char &splitChar)
    {
        //Declare Variables:
        std::vector<std::string> sVec;
        std::string statement = "";

        //Traverse str:
        for (unsigned int i = 0; i < str.size(); ++i)
        {
            //If Iteration != splitChar:
            if (str.at(i) != splitChar)
            {
                //Add It To statement:
                statement += str.at(i);
            }
            //If Iteration == splitChar:
            else if (str.at(i) == splitChar)
            {
                //Push Vector With statement
                sVec.push_back(statement);
                //Reset Statement:
                statement = "";
            }

            if (i == str.size() - 1)
            {
                //Push Vector With statement
                sVec.push_back(statement);
            }
        }

        return sVec;
    }

    static std::string RemoveChar(const std::string &str, const char &rem)
    {
        //Declare Variable:
        std::string newStr = "";

        //Traverse str:
        for (int i = 0; i < str.length(); i++)
        {
            //If Iteration != rem:
            if (str.at(i) != rem)
            {
                //Add It To newWord:
                newStr += str.at(i);
            }
        }
        //Return newWord:
        return newStr;
    }

    static void removeSubstrs(std::string &s, std::string &p)
    {
        std::string::size_type n = p.length();
        for (std::string::size_type i = s.find(p);
             i != std::string::npos;
             i = s.find(p))
            s.erase(i, n);
    }

};


#endif // STRING_METHODS_H
