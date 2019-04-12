#ifndef MACELOG_H
#define MACELOG_H

#include "iostream"
#include "termcolor.hpp"

class MaceLog
{
public:
    MaceLog() {}

    // Message type wrappers:
    static void Error(const std::string &text) { std::cout << termcolor::red << text << termcolor::reset << std::endl; }
    static void Warning(const std::string &text) { std::cout << termcolor::yellow << text << termcolor::reset << std::endl; }
    static void Info(const std::string &text) { std::cout << termcolor::cyan << text << termcolor::reset << std::endl; }
    static void Alert(const std::string &text) { std::cout << termcolor::magenta << text << termcolor::reset << std::endl; }
    static void Notice(const std::string &text) { std::cout << termcolor::grey << text << termcolor::reset << std::endl; }
    static void Debug(const std::string &text) { std::cout << termcolor::green << text << termcolor::reset << std::endl; }
    static void Critical(const std::string &text) { std::cout << termcolor::on_yellow << text << termcolor::reset << std::endl; }
    static void Emergency(const std::string &text) { std::cout << termcolor::on_red << text << termcolor::reset << std::endl; }

    // Foreground Color wrappers:
    static void Grey(const std::string &text) { std::cout << termcolor::grey << text << termcolor::reset << std::endl; }
    static void Red(const std::string &text) { std::cout << termcolor::red << text << termcolor::reset << std::endl; }
    static void Green(const std::string &text) { std::cout << termcolor::green << text << termcolor::reset << std::endl; }
    static void Yellow(const std::string &text) { std::cout << termcolor::yellow << text << termcolor::reset << std::endl; }
    static void Blue(const std::string &text) { std::cout << termcolor::blue << text << termcolor::reset << std::endl; }
    static void Magenta(const std::string &text) { std::cout << termcolor::magenta << text << termcolor::reset << std::endl; }
    static void Cyan(const std::string &text) { std::cout << termcolor::cyan << text << termcolor::reset << std::endl; }
    static void White(const std::string &text) { std::cout << termcolor::white << text << termcolor::reset << std::endl; }

    // Background Color wrappers:
    static void bgGrey(const std::string &text) { std::cout << termcolor::on_grey << text << termcolor::reset << std::endl; }
    static void bgRed(const std::string &text) { std::cout << termcolor::on_red << text << termcolor::reset << std::endl; }
    static void bgGreen(const std::string &text) { std::cout << termcolor::on_green << text << termcolor::reset << std::endl; }
    static void bgYellow(const std::string &text) { std::cout << termcolor::on_yellow << text << termcolor::reset << std::endl; }
    static void bgBlue(const std::string &text) { std::cout << termcolor::on_blue << text << termcolor::reset << std::endl; }
    static void bgMagenta(const std::string &text) { std::cout << termcolor::on_magenta << text << termcolor::reset << std::endl; }
    static void bgCyan(const std::string &text) { std::cout << termcolor::on_cyan << text << termcolor::reset << std::endl; }
    static void bgWhite(const std::string &text) { std::cout << termcolor::on_white << termcolor::blue << text << termcolor::reset << std::endl; }

};

#endif // MACELOG_H
