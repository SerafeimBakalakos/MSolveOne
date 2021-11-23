using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.IsoXFEM
{
    public class Node
    {
        /// <summary>
        /// The ID of the node. ID should not be a negative number.
        /// </summary>
        public int ID { get; }
        /// <summary>
        /// X-coordinate of the node
        /// </summary>
        public double X { get; }
        /// <summary>
        /// Y-coordinate of the node
        /// </summary>
        public double Y { get; }
        /// <summary>
        /// X-constrain of the node
        /// </summary>
        public bool IsXConstrained { get; }
        /// <summary>
        /// Y-constrain of the node
        /// </summary>
        public bool IsYConstrained { get; }
        /// <summary>
        /// Elements that include this Node
        /// </summary>
        public List<Element> elementsOnNode = new List<Element>();		
		/// <summary>
		/// Constructs a Node objects and initializes it with its ID and X any Y coordinates.
		/// </summary>
		/// <param name="id">The ID of the node (should not be a negative number)</param>
		/// <param name="x">X coordinate of the node</param>
		/// <param name="y">Y coordinate of the node</param>
		/// <param name="isXConstrained">X-constrain of the node</param>
		/// <param name="isYConstrained">Y-constrain of the node</param>
		public Node(int id, double x, double y, bool isXConstrained, bool isYConstrained)
        {
            ID = id >= 0 ? id:-id;
            X = x;
            Y = y;
            IsXConstrained = isXConstrained;
            IsYConstrained = isYConstrained;
        } 
        
    }
}
