using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.IsoXFEM
{
    public class GeometryProperties
    {
        /// <summary>
        /// The length of the model.
        /// </summary>
        public double length;
        /// <summary>
        /// The height of the model.
        /// </summary>
        public double height;
        /// <summary>
        /// The thickness of the model.
        /// </summary>
        public double thickness;
		/// <summary>
		/// The number of elements on each axis.
		/// </summary>
		public int [] NumberOfElementsOnAxes { get; }    
		/// <summary>
		/// Constructs a Geometry object and initializes it with its length, height and thickness dimensions and numberOfElements X,Y and Z  in order to create the mesh of the model.
		/// </summary>
		/// <param name="length"> The length of the model</param>
		/// <param name="height"> The height of the model</param>
		/// <param name="thickness"> The thickness of the model</param>
		/// <param name="numberOfElementsOnAxes"> The number of elements on each axis.</param>
		public GeometryProperties(double length, double height, double thickness, params int [] numberOfElementsOnAxes)
        {
            this.length = length;
            this.height = height;
            this.thickness= thickness;
			this.NumberOfElementsOnAxes = numberOfElementsOnAxes;
		}
		/// <summary>
		/// The number of elements on X dimension.
		/// </summary>
		public int NumberOfElementsX => NumberOfElementsOnAxes[0];
		/// <summary>
		/// The number of elements on Y dimension.
		/// </summary>
		public int NumberOfElementsY => NumberOfElementsOnAxes[1];
		/// <summary>
		/// The number of elements on Z dimension.
		/// </summary>
		public int NumberOfElementsZ => NumberOfElementsOnAxes[2];
	}
}
