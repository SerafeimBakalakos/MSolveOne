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
        /// The number of elements on X dimension.
        /// </summary>
        public int numberOfElementsX;
        /// <summary>
        /// The number of elements on Y dimension.
        /// </summary>
        public int numberOfElementsY;
		/// <summary>
		/// The number of elements on Z dimension.
		/// </summary>
		public int numberOfElementsZ;
		/// <summary>
		/// Constructs a Geometry object and initializes it with its length, height and thickness dimensions and numberOfElements X and Y in order to create the mesh of the model for 2D applications.
		/// </summary>
		/// <param name="length"> The length of the model</param>
		/// <param name="height"> The height of the model</param>
		/// <param name="thickness"> The thickness of the model</param>
		/// <param name="numberOfElementsX"> The number of elements on X dimension</param>
		/// <param name="numberOfElementsY"> The number of elements on Y dimension</param>
		public GeometryProperties(double length, double height, double thickness, int numberOfElementsX, int numberOfElementsY)
        {
            this.length = length;
            this.height = height;
            this.thickness= thickness;
            this.numberOfElementsX = numberOfElementsX;
            this.numberOfElementsY= numberOfElementsY;
        }
		/// <summary>
		/// Constructs a Geometry object and initializes it with its length, height and thickness dimensions and numberOfElements X, Y and Z in order to create the mesh of the model for 3D applications.
		/// </summary>
		/// <param name="length"> The length of the model</param>
		/// <param name="height"> The height of the model</param>
		/// <param name="thickness"> The thickness of the model</param>
		/// <param name="numberOfElementsX"> The number of elements on X dimension</param>
		/// <param name="numberOfElementsY"> The number of elements on Y dimension</param>
		/// <param name="numberOfElementsZ"> The number of elements on Y dimension</param>
		public GeometryProperties(double length, double height, double thickness, int numberOfElementsX, int numberOfElementsY, int numberOfElementsZ)
		{
			this.length = length;
			this.height = height;
			this.thickness = thickness;
			this.numberOfElementsX = numberOfElementsX;
			this.numberOfElementsY = numberOfElementsY;
			this.numberOfElementsZ = numberOfElementsZ;
		}
    }
}
