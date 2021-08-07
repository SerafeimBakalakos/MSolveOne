using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Exceptions
{
    public class InvalidElementGeometryIntersectionException : Exception
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="InvalidElementGeometryIntersectionException"/> class.
        /// </summary>
        public InvalidElementGeometryIntersectionException()
        { }

        /// <summary>
        /// Initializes a new instance of the <see cref="InvalidElementGeometryIntersectionException"/> class with a specified 
        /// error message.
        /// </summary>
        /// <param name="message">The error message that explains the reason for the exception.</param>
        public InvalidElementGeometryIntersectionException(string message) : base(message)
        { }

        /// <summary>
        /// Initializes a new instance of the <see cref="InvalidElementGeometryIntersectionException"/> class with a 
        /// specified error message and a reference to the inner exception that is the cause of this exception.
        /// </summary>
        /// <param name="message">The error message that explains the reason for the exception.</param>
        /// <param name="inner">The exception that is the cause of the current exception. If the innerException parameter is not 
        ///     a null reference, the current exception is raised in a catch block that handles the inner exception. </param>
        public InvalidElementGeometryIntersectionException(string message, Exception inner) : base(message, inner)
        { }
    }
}
