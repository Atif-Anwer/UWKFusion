using System;
using System.IO;

namespace UWKFusion.Logging
{
    class FileAccess
    {
        private StreamReader filereader;
        private StreamWriter filewriter;


        public static FileAccess OpenFileForRead(string filename)
        {
            StreamReader sr = new StreamReader(filename);
            return new FileAccess(sr, null);


        }

        public static FileAccess OpenFileForWrite(string filename)
        {
            StreamWriter sr = new StreamWriter(filename);
            return new FileAccess(null, sr);
        }

        private FileAccess(StreamReader reader, StreamWriter writer)
        {
            filereader = reader;
            filewriter = writer;

        }

        public string ReadLine()
        {
            if (filereader != null)
            {
                return filereader.ReadLine();
            }
            else
            {
                throw new InvalidOperationException("File open fo read");
            }
        }

        public void WriteLine(string message)
        {
            if (filewriter != null)
            {
                filewriter.WriteLine(message);
            }
            else
            {
                throw new InvalidOperationException("File open fo write");
            }
        }

        public void Dispose()
        {
            if (filereader != null)
            {
                filereader.Close();
                filereader.Dispose();
            }

            if (filewriter != null)
            {
                filewriter.Close();
                filewriter.Dispose();
            }
        }
    }
}
