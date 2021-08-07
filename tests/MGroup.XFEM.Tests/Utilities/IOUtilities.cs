using System;
using System.IO;
using MGroup.MSolve.DataStructures;

namespace MGroup.XFEM.Tests.Utilities
{
    /// <summary>
    /// Utility methods for IO and file operations.
    /// Authors: Serafeim Bakalakos
    /// </summary>
    internal static class IOUtilities
    {
        /// <summary>
        /// Compares 2 files line-by-line, taking into account equivalent characters. If double numbers are encountered, use 
        /// tolerance to compare them.
        /// </summary>
        /// <param name="file1">The absolute path of the first file.</param>
        /// <param name="file2">The absolute path of the second file.</param>
        /// <param name="tolerance">
        /// The tolerance to compare any double numbers: |computed-expected|/|expected| &lt; <paramref name="tolerance"/>.
        /// </param>
        internal static bool AreDoubleValueFilesEquivalent(string file1, string file2, double tolerance)
        {
            var comparer = new ValueComparer(tolerance);

            // Determine if the same file was referenced two times.
            if (file1 == file2)
            {
                // Return true to indicate that the files are the same.
                return true;
            }
            // Open the two files.
            using (var reader1 = new StreamReader(file1))
            using (var reader2 = new StreamReader(file2))
            {
                while (true)
                {
                    // Reading lines removes the \r, \n, or \r\n at the end.
                    string line1 = reader1.ReadLine();
                    string line2 = reader2.ReadLine();

                    if ((line1 == null) && (line2 == null))
                    {
                        return true; // both files have ended without finding a difference.
                    }
                    else if ((line1 == null) != (line2 == null))
                    {
                        return false; // only 1 file has ended.
                    }
                    
                    var words1 = line1.Split(' ', ',');
                    bool valueLine = double.TryParse(words1[0], out double number);
                    if (valueLine)
                    {
                        // Compare the data in this line as double numbers
                        var words2 = line2.Split(' ', ',');
                        if (words1.Length != words2.Length)
                        {
                            return false;
                        }
                        for (int i = 0; i < words1.Length; ++i)
                        {
                            double value1 = double.Parse(words1[i]);
                            double value2 = double.Parse(words2[i]);
                            if (!comparer.AreEqual(value1, value2))
                            {
                                return false;
                            }
                        }

                    }
                    else if (!line1.Trim().Equals(line2.Trim())) // Compare the lines as strings
                    {
                        return false;
                    }
                }
            }
        }
    }
}
