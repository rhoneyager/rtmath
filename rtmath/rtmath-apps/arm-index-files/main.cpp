#include <iostream>
#include <fstream>
#include <boost/asio/ip/host_name.hpp> // for system host name
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <netcdf.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include <sstream>
#include <libpq-fe.h>
#include <pqxx/pqxx>
#include "../../rtmath/rtmath/config.h"
#include "../../rtmath/rtmath/command.h"
void doHelp();

int main(int argc, char** argv)
{
        try {
                using namespace std;
                set<string> files;
                cerr << "arm-preprocess" << endl;

                // Begin by loading parameters from command line
                // Is help being called?
                if (argc == 1) doHelp();
                // Flags go here. The flag setting code is below.
                bool clobberFiles = false;
                bool uncompress = false;
                string ppath;
                string sdbconn;
                {
                        bool flag = false;
                        rtmath::config::parseParams p(argc,argv);
                        flag = p.readParam("-h");
                        if (flag) doHelp();

                        // Get the directory or file to be processed
                        // If not specified, produce help and exit
                        flag = p.readParam<string>("-f", ppath);
                        if (!flag) doHelp();

                        // Shall old file entries be clobbered?
                        clobberFiles = p.readParam("-clobber");

                        // Shall .cdf.bz2 files be indexed as well?
                        uncompress = p.readParam("-u");

                        // Get database connection information from either
                        // command line or from user rtmath configuration
                        // files. Database class has an easy loader
                        // function that parses the command line appropriately.
                        string fconfig;
                        rtmath::config::getConfigDefaultFile(fconfig);
                        p.readParam<string>("-config", fconfig);
			std::shared_ptr<rtmath::config::configsegment> croot;
//                        rtmath::config::configsegment *croot;
                        croot = rtmath::config::configsegment::loadFile(fconfig.c_str(),NULL);
                        string database = "arm";
                        string username, password, host;
                        string sslmode = "require";
                        string port = "5432";
                        host = boost::asio::ip::host_name();
                        croot->getVal("arm/database", database);
                        croot->getVal("arm/host", host);
                        croot->getVal("arm/username", username);
                        croot->getVal("arm/password", password);
                        croot->getVal("arm/sslmode", sslmode);
                        croot->getVal("arm/port", port);

                        ostringstream ssdbconn;
                        if (host != "local")
                                ssdbconn << "host=" << host << " " <<
                                        "port=" << port << " " <<
                                        "sslmode=" << sslmode << " ";
                        ssdbconn << "dbname=" << database << " " <<
                                "user=" << username << " " <<
                                "password=" << password << " ";
                        sdbconn = ssdbconn.str();
                }

                // Connect to the database. If the connection fails,
                // die here.
                // cout << "Connecting with " << sdbconn << endl;
                cout << "Connecting to database\n";
                pqxx::connection c(sdbconn);
                // pqxx::work txn(c);

                // Create a set containing all valid files
                // If only one file, then it's easy
                // If a directory, select *.nc
                // By the way, verify file / directory existence here
                // The console-provided data is at ppath
                // The file list is files
                {
                        using namespace boost::filesystem;
                        path root(ppath.c_str());
                        // Is ppath a directory or a filename?
                        if (!exists(root))
                        {
                                cerr << "Path " << ppath <<
                                        " does not exist!\n";
                                return 1;
                        }
                        if (is_directory(root))
                        {
                                // If a directory, recurse and select
                                // all netcdf files.
                                vector<path> dirc;
                                copy(recursive_directory_iterator(root),
                                                recursive_directory_iterator(),
                                                back_inserter(dirc));
                                for (auto it = dirc.begin(); it != dirc.end(); it++)
                                {
                                        if (it->extension().string() == ".bz2")
                                        {
                                                // Check if it is a compressed
                                                // netCDF file. If so, uncompress
                                                path realfile = it->stem();
                                                if (realfile.extension().string() == "cdf")
                                                {
                                                        // Check for uncompress option
                                                        if (uncompress)
                                                        {
                                                                // Uncompress the file
                                                                //throw; // for now
                                                                string a = it->string();
                                                                cout << "Decompressing " << a << endl;

                                                                execl("/usr/bin/bunzip2", a.data());
                                                                files.insert(realfile.string());

                                                        } else {
                                                                cout << "Skipping file " << it->string() << endl;
                                                        }
                                                }
                                        }
                                        if (it->extension().string() == ".cdf")
                                        {
                                                files.insert(it->string());
                                        }
                                }
                        } else
                        {
                                // If a filename, is it a netcdf file?
                                if (root.extension().string() == ".cdf")
                                {
                                        files.insert(ppath);
                                } else {
                                        cerr << "File " << ppath <<
                                                " is not a netCDF file.\n";
                                        return 1;
                                }
                        }

                        if (files.size() != 1)
                                cout << "Processing " << files.size() << " files.\n";
                        else
                                cout << "Processing 1 file.\n";
                        if (files.size() == 0)
                        {
                                cerr << "No valid files to process\n";
                                return 1;
                        }
                }

                boost::uuids::random_generator ugen;
                // Actually loop through the file here
                // Let's use the new for_each
                //for ( string &fname : files )
                int numAdded = 0;
                for (auto it=files.begin();it!=files.end();it++)
                //if (1==0)
                {
                        // Note that the absolute file path is not a good
                        // choice for uniqueness due to symlinks and file
                        // moving. Thankfully, ARM ensures that file names
                        // are unique.
                        using namespace boost::filesystem;
                        path fpath = path(*it);
                        path fname = fpath.filename();
                        // Consult with database to see if file has
                        // been previously processed
                        pqxx::work txn(c); // Perform as a transaction
                        pqxx::result r = txn.exec(
                                        "SELECT filename "
                                        "FROM files "
                                        "WHERE filename ="
                                        + txn.quote(fname.string().c_str())
                                        );
                        if (r.size() != 0)
                        {
                                // A duplicate file entry exists.
                                // See if it can be clobbered.
                                // If not, then fail with error.
                                continue;
                        }
                        numAdded++;
                        unsigned long fsize = file_size(fpath);
                        //cout << endl << "File " << fname << " has " << fsize << " bytes.\n";

                        // uuid generation
                        string uid;
                        boost::uuids::uuid u = ugen();
                        uid = boost::uuids::to_string(u);
                        //cout << "uuid " << uid << endl;

                        // Open the file
                        int error = 0;
                        int nfile = -1;
                        error = nc_open(it->c_str(), NC_NOWRITE, &nfile);
                        if (error) { cerr << "Error opening netcdf file " << fname << endl; continue; }

                        try {

                                // Load the dimension / variable data
                                // I care about the time dimension, the base_time variable
                                int d_time, v_time, v_timeoff;
                                size_t numtimes = 0;
                                nc_inq_dimid(nfile,"time",&d_time);
                                nc_inq_dimlen(nfile,d_time, &numtimes);
                                nc_inq_varid(nfile,"base_time",&v_time);
                                nc_inq_varid(nfile,"time_offset",&v_timeoff);

                                int time_start, time_stop;
                                nc_get_var1_int(nfile,v_time,0,&time_start);
                                size_t numtimesm = numtimes - 1;
                                nc_get_var1_int(nfile,v_timeoff,&numtimesm,&time_stop);
                                // Convert these to text strings.
                                string ststart, ststop;
                                // By convention, these are seconds since midnight
                                // 1970-01-01 in UTC
                                {
                                        using namespace boost::posix_time;
                                        using namespace boost::gregorian;
                                        time_duration tdstart(0,0,time_start,0);
                                        time_duration tdstop(0,0,time_stop+time_start,0);
                                        ptime pstart(date(1970,Jan,1),tdstart);
                                        ptime pstop(date(1970,Jan,1),tdstop);
                                        ststart = to_simple_string(pstart);
                                        ststop = to_simple_string(pstop);
                                        //cout << "Start " << ststart << endl;
                                        //cout << "Stop " << ststop << endl;
                                }

                                // Get site id
                                // Site id global attribute is site_id
                                // facility_id is the extended id
                                // mixed id is zeb_platform (but in all?)
                                // other id in command_line
                                string sid = "NUL", fid = "NUL";
                                {
                                        // Use alternate method for getting site and
                                        // facility, as the netcdf vars are
                                        // unreliable across datastreams.
                                        // Site is first three characters of filename
                                        // Facility is the two characters before the first .
                                        string name = fname.string();
                                        sid = name.substr(0,3);
                                        size_t ploc = name.find(".");
                                        fid = name.substr(ploc-2,2);
                                        //cout << "Site " << sid << endl;
                                        //cout << "Facility " << fid << endl;
                                }

                                // Get instrument
                                // not in any standard format
                                string instrument;
                                {
                                        // To find, take filename between site id and facility id
                                        // Ex: sbsvceil25kM1.... site is sbs, facility is M1
                                        string name = fname.string();
                                        // All sites are three letters long
                                        size_t floc = name.find(fid);
                                        instrument = name.substr(3,floc-3);
                                        //cout << "Stream " << instrument << endl;
                                }

                                // Have the database create the file entry!
                                {
                                        using namespace boost::filesystem;
                                        path abspath = absolute(fpath);
                                        ostringstream ins;
                                        ins << "INSERT INTO files " <<
                                                "(id,filename,path,size,site,facility,stream," <<
                                                "time_start,time_stop,time_duration)" <<
                                                " VALUES ('" <<
                                                uid << "','" << fname.string() << "','" << abspath.string()  <<
                                                "'," << fsize << ",'" << sid << "','" << fid << "','" <<
                                                instrument << "','" <<
                                                ststart << "','" << ststop << "'," << time_stop <<
                                                ")";
                                        //cout << ins.str() << endl;
                                        txn.exec(ins.str().c_str());
                                        txn.commit();

                                        // Check result for errors.
                                        // If any, report and die.
                                }
                        } // end try block for cdf and database
                        catch (...)
                        {
                                cout << "ERROR processing file" << endl;
                        }

                        // And we're done with the file
                        // netcdf close
                        nc_close(nfile);
                }

                cout << numAdded << " files added to table files.\n";
                // Close the database connection

        }
        catch (rtmath::debug::xError &err)
        {
                err.Display();
                std::cerr << std::endl;
                return 1;
        }
        // Once here, we're done with the processing
        return 0;
}

void doHelp()
{
        using namespace std;
        cout << "Options:\n";
        cout << "-f (file/folder name)\n";
        cout << "\tSpecify the name of a file or folder\n";
        cout << "\tto import. Folders are analyzed to \n";
        cout << "\tdepth 1. Files are handled individually.\n";
        cout << "-u\n";
        cout << "\tIf bzip-compressed cdf files found in \n";
        cout << "\tfolder, decompress them and analyze.\n";
        cout << "-clobber\n";
        cout << "\tReanalyze files already in the database.\n";
        exit(1);
}

