#ifndef MACELOG_H
#define MACELOG_H

#include "iostream"
#include "termcolor.hpp"
#include "spdlog/spdlog.h"
#include "spdlog/async.h" //support for async logging.
#include "spdlog/sinks/basic_file_sink.h"

class MaceLog
{
public:
    MaceLog() {}
    ~MaceLog() = default;

    // **************************************************************** //
    // Virtual logging methods:
    // **************************************************************** //
public:
    virtual void logToFile(std::shared_ptr<spdlog::logger> logger, const std::string &str) {
        if(logger) {
            logger->info(str);
        }
    }


    // **************************************************************** //
    // In-line console logging with color support:
    // **************************************************************** //
public:
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

    // Foreground Color wrappers (inline):
    static void Grey_no_newline(const std::string &text) { std::cout << termcolor::grey << text << termcolor::reset; }
    static void Red_no_newline(const std::string &text) { std::cout << termcolor::red << text << termcolor::reset; }
    static void Green_no_newline(const std::string &text) { std::cout << termcolor::green << text << termcolor::reset; }
    static void Yellow_no_newline(const std::string &text) { std::cout << termcolor::yellow << text << termcolor::reset; }
    static void Blue_no_newline(const std::string &text) { std::cout << termcolor::blue << text << termcolor::reset; }
    static void Magenta_no_newline(const std::string &text) { std::cout << termcolor::magenta << text << termcolor::reset; }
    static void Cyan_no_newline(const std::string &text) { std::cout << termcolor::cyan << text << termcolor::reset; }
    static void White_no_newline(const std::string &text) { std::cout << termcolor::white << text << termcolor::reset; }


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
