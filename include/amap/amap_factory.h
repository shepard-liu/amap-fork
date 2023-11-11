#ifndef PROJECT_AMAP_FACTORY_H
#define PROJECT_AMAP_FACTORY_H

#include <iostream>

#include "geodetic_conv.h"

#include "amap_common.h"
#include "amap_io.h"
#include "amap_io_alpha.h"
#include "amap_io_beta.h"
#include "amap_io_gamma.h"

namespace alive
{
    class AliveMapFileIOFactory
    {
    public:
        static AliveMapFileLoaderPtr getLoader(const std::string &path)
        {
            AliveMapFileLoaderPtr res;

            AliveMapHeader header;
            std::fstream fs;
            fs.open(path.data(), std::ios::in | std::ios::binary);

            if (!fs.is_open())
            {
                std::cout << "CANNOT OPEN" << std::endl;
                return nullptr;
            }

            // read header first
            fs.read(reinterpret_cast<char *>(&header), sizeof(AliveMapHeader));

            fs.close();

            // alpha-version map
            if (header.file_signature[0] == 'a')
            {
                AliveMapFileAlphaLoaderPtr loader(new AliveMapFileAlphaLoader());
                res = loader;
            }
            // beta-version map
            else if (header.file_signature[0] == 'b')
            {
                AliveMapFileBetaLoaderPtr loader(new AliveMapFileBetaLoader());
                res = loader;
            }
            // gamma-version map
            else if (header.file_signature[0] == 'g')
            {
                AliveMapFileGammaLoaderPtr loader(new AliveMapFileGammaLoader());
                res = loader;
            }
            else
            {
                std::cerr << "no alive map loader is not supported" << std::endl;
                std::abort();
            }

            return res;
        }

        static AliveMapFileSaverPtr getSaver(int &type)
        {
            AliveMapFileSaverPtr res = nullptr;

            // alpha-version map
            if (type == 1)
            {
                AliveMapFileAlphaSaverPtr saver(new AliveMapFileAlphaSaver());
                res = saver;
            }
            // bate-version map
            else if (type == 2)
            {
                // todo
                AliveMapFileBetaSaverPtr saver(new AliveMapFileBetaSaver());
                res = saver;
            }
            // gamma-version map
            else if (type == 3)
            {
                AliveMapFileGammaSaverPtr saver(new AliveMapFileGammaSaver());
                res = saver;
            }
            else
            {
                std::cerr << "no map saver type is supported" << std::endl;
                std::abort();
            }

            return res;
        }
    };
} // namespace alive

#endif