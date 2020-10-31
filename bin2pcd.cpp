/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011-2012, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */
#include <fstream> // for ifstream

#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include "./point_data.h"

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

namespace pcl {

	class BinReader: public FileReader {
	public:
		/** \brief Read a point cloud data header from a FILE file. 
		 *
		 * Load only the meta information (number of points, their types, etc),
		 * and not the points themselves, from a given FILE file. Useful for fast
		 * evaluation of the underlying data structure.
		 *
		 * Returns:
		 *  * < 0 (-1) on error
		 *  * > 0 on success
		 * \param[in] file_name the name of the file to load
		 * \param[out] cloud the resultant point cloud dataset (only the header will be filled)
		 * \param[out] origin the sensor acquisition origin (only for > FILE_V7 - null if not present)
		 * \param[out] orientation the sensor acquisition orientation (only for > FILE_V7 - identity if not present)
		 * \param[out] file_version the FILE version of the file (either FILE_V6 or FILE_V7)
		 * \param[out] data_type the type of data (binary data=1, ascii=0, etc)
		 * \param[out] data_idx the offset of cloud data within the file
		 * \param[in] offset the offset in the file where to expect the true header to begin.
		 * One usage example for setting the offset parameter is for reading
		 * data from a TAR "archive containing multiple files: TAR files always
		 * add a 512 byte header in front of the actual file, so set the offset
		 * to the next byte after the header (e.g., 513).
		 */
		virtual int readHeader (const std::string &file_name,
				pcl::PCLPointCloud2 &cloud,
				Eigen::Vector4f &origin,
				Eigen::Quaternionf &orientation,
				int &file_version,
				int &data_type,
				unsigned int &data_idx,
				const int offset = 0) override
		{
			return 0;
		}

		/** \brief Read a point cloud data from a FILE file and store it into a pcl/PCLPointCloud2.
		 * \param[in] file_name the name of the file containing the actual PointCloud data
		 * \param[out] cloud the resultant PointCloud message read from disk
		 * \param[out] origin the sensor acquisition origin (only for > FILE_V7 - null if not present)
		 * \param[out] orientation the sensor acquisition orientation (only for > FILE_V7 - identity if not present)
		 * \param[out] file_version the FILE version of the file (either FILE_V6 or FILE_V7)
		 * \param[in] offset the offset in the file where to expect the true header to begin.
		 * One usage example for setting the offset parameter is for reading
		 * data from a TAR "archive containing multiple files: TAR files always
		 * add a 512 byte header in front of the actual file, so set the offset
		 * to the next byte after the header (e.g., 513).
		 */
		virtual int read (const std::string &file_name,
				pcl::PCLPointCloud2 &cloud,
				Eigen::Vector4f &origin,
				Eigen::Quaternionf &orientation,
				int &file_version,
				const int offset = 0) override
		{
			std::ifstream ist(file_name.c_str(), std::ios::in | std::ios::binary);
			for (;;) {
				point_data_t data;
				ist.read(reinterpret_cast<char*>(&data), sizeof(data));
				if(ist.eof()) {
					break;
				}
				PointCloud<PointXYZRGB> point;
				point.push_back(PointXYZRGB(data.x, data.y, data.z, data.r, data.g, data.b));
				PCLPointCloud2 pcl2;
				toPCLPointCloud2(point, pcl2);
				cloud += pcl2;
			}
		}

		using FileReader::read;
	};

}

void printHelp (int, char **argv)
{
	print_error ("Syntax is: %s [-format 0|1] input.bin output.pcd\n", argv[0]);
}

bool loadCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud)
{
	TicToc tt;
	print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

	pcl::BinReader reader;
	tt.tic ();
	if (reader.read (filename, cloud) < 0)
		return (false);
	print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
	print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

	return (true);
}

void saveCloud (const std::string &filename, const pcl::PCLPointCloud2 &cloud, bool format)
{
	TicToc tt;
	tt.tic ();

	print_highlight ("Saving "); print_value ("%s ", filename.c_str ());

	pcl::PCDWriter writer;
	writer.write (filename, cloud, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), format);

	print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
}

/* ---[ */
int main (int argc, char** argv)
{
	print_info ("Convert a BIN file(position,rgba) to PCD format. For more information, use: %s -h\n", argv[0]);

	if (argc < 3)
	{
		printHelp (argc, argv);
		return (-1);
	}

	// Parse the command line arguments for .pcd and .bin files
	std::vector<int> pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
	std::vector<int> bin_file_indices = parse_file_extension_argument (argc, argv, ".bin");
	if (pcd_file_indices.size () != 1 || bin_file_indices.size () != 1)
	{
		print_error ("Need one input RAW file and one output PCD file.\n");
		return (-1);
	}

	// Command line parsing
	bool format = true;
	parse_argument (argc, argv, "-format", format);
	print_info ("PCD output format: "); print_value ("%s\n", (format ? "binary" : "ascii"));

	// Load the first file
	pcl::PCLPointCloud2 cloud;
	if (!loadCloud (argv[bin_file_indices[0]], cloud)) 
		return (-1);

	// Convert to PCD and save
	saveCloud (argv[pcd_file_indices[0]], cloud, format);

	return (0);
}

