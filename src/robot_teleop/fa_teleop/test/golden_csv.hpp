#pragma once

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fa_teleop::test
{
    /// Read a numeric CSV fixture ('#' lines are comments; "nan" allowed).
    inline std::vector<std::vector<double>> readCsv(const std::string& path)
    {
        std::ifstream in(path);
        if (!in)
        {
            throw std::runtime_error("cannot open golden file: " + path);
        }
        std::vector<std::vector<double>> rows;
        std::string line;
        while (std::getline(in, line))
        {
            if (line.empty() || line[0] == '#')
            {
                continue;
            }
            std::vector<double> row;
            std::stringstream ss(line);
            std::string cell;
            while (std::getline(ss, cell, ','))
            {
                row.push_back(std::strtod(cell.c_str(), nullptr));
            }
            rows.push_back(std::move(row));
        }
        return rows;
    }

    inline std::string dataPath(const std::string& name)
    {
        return std::string(FA_TELEOP_TEST_DATA_DIR) + "/" + name;
    }
} // namespace fa_teleop::test
