using System;
using System.Collections.Generic;
using System.Dynamic;
using System.Linq;
using System.Text;
using MGroup.LinearAlgebra.Commons;
using MGroup.LinearAlgebra.Exceptions;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.Solvers.DDM.Prototypes.LinearAlgebraExtensions
{
    public class BlockMatrix : IVectorMultipliable
    {
        private readonly int numRowBlocks;
        private readonly int numColBlocks;
        private readonly Dictionary<int, int> rowsPerBlock;
        private readonly Dictionary<int, int> colsPerBlock;
        private readonly Matrix[,] blocks;

        private BlockMatrix(IDictionary<int, int> rowsPerBlock, IDictionary<int, int> colsPerBlock)
        {
            if (!Utilities.AreIndices(rowsPerBlock.Keys))
            {
                throw new ArgumentException(
                    "The keys of the provided row dictionary must be indices, namely be unique and belong to [0, count)");
            }
            if (!Utilities.AreIndices(colsPerBlock.Keys))
            {
                throw new ArgumentException(
                    "The keys of the provided column dictionary must be indices, namely be unique and belong to [0, count)");
            }

            this.rowsPerBlock = new Dictionary<int, int>();
            foreach (var blockIdxLengthPair in rowsPerBlock)
            {
                int blockIdx = blockIdxLengthPair.Key;
                int blockLength = blockIdxLengthPair.Value;
                ++this.numRowBlocks;
                this.rowsPerBlock[blockIdx] = blockLength;
                this.NumRows += blockLength;

            }

            this.colsPerBlock = new Dictionary<int, int>();
            foreach (var blockIdxLengthPair in colsPerBlock)
            {
                int blockIdx = blockIdxLengthPair.Key;
                int blockLength = blockIdxLengthPair.Value;
                ++this.numColBlocks;
                this.colsPerBlock[blockIdx] = blockLength;
                this.NumColumns += blockLength;

            }

            this.blocks = new Matrix[numRowBlocks, numColBlocks];
        }

        public int NumColumns { get; }

        public int NumRows { get; }

        public int[][] RowMultiplicities { get; set; }

        public int[][] ColMultiplicities { get; set; }

        public static BlockVector operator *(BlockMatrix matrix, BlockVector vector)
        {
            Preconditions.CheckMultiplicationDimensions(matrix.numColBlocks, vector.Blocks.Length);
            Preconditions.CheckMultiplicationDimensions(matrix.NumColumns, vector.Length);
            var result = new BlockVector(matrix.rowsPerBlock);
            result.Multiplicities = matrix.RowMultiplicities;
            for (int i = 0; i < matrix.numRowBlocks; ++i)
            {
                var rowResult = Vector.CreateZero(matrix.rowsPerBlock[i]);
                result.AddBlock(i, rowResult);
                for (int j = 0; j < matrix.numColBlocks; ++j)
                {
                    if (matrix.blocks[i, j] != null)
                    {
                        rowResult.AddIntoThis(matrix.blocks[i, j] * vector.Blocks[j]);
                    }
                }
            }
            return result;
        }

        public static BlockVector operator *(BlockMatrix matrix, Vector vector) => matrix * new BlockVector(vector);

        public static BlockMatrix Create(IDictionary<int, int> rowsPerBlock, IDictionary<int, int> colsPerBlock)
            => new BlockMatrix(rowsPerBlock, colsPerBlock);

        public static BlockMatrix CreateCol(IDictionary<int, int> rowsPerBlock, int numColsTotal)
        {
            var colsPerBlock = new Dictionary<int, int>();
            colsPerBlock[0] = numColsTotal;
            return new BlockMatrix(rowsPerBlock, colsPerBlock);
        }

        public static BlockMatrix CreateRow(int numRowsTotal, IDictionary<int, int> colsPerBlock)
        {
            var rowsPerBlock = new Dictionary<int, int>();
            rowsPerBlock[0] = numRowsTotal;
            return new BlockMatrix(rowsPerBlock, colsPerBlock);
        }

        public static BlockMatrix CreateSingle(Matrix matrix)
        {
            var rowsPerBlock = new Dictionary<int, int>();
            rowsPerBlock[0] = matrix.NumRows;
            var colsPerBlock = new Dictionary<int, int>();
            colsPerBlock[0] = matrix.NumColumns;
            var result = new BlockMatrix(rowsPerBlock, colsPerBlock);
            result.blocks[0, 0] = matrix;
            return result;
        }

        public void AddBlock(int rowBlockIdx, int colBlockIdx, Matrix block)
        {
            // Check that dimensions are compatible with the other blocks
            int numRows = rowsPerBlock[rowBlockIdx];
            if ((numRows > 0) && (block.NumRows != numRows))
            {

                throw new NonMatchingDimensionsException(
                    $"The existing blocks of the same block row have {numRows} rows," +
                    $" but the new block has {block.NumRows} rows");
            }

            int numCols = colsPerBlock[colBlockIdx];
            if ((numCols > 0) && (block.NumColumns != numCols))
            {

                throw new NonMatchingDimensionsException(
                    $"The existing blocks of the same block column have {numCols} columns," +
                        $" but the new block has {block.NumColumns} columns");
            }

            blocks[rowBlockIdx, colBlockIdx] = block;
        }

        public Matrix CopyToFullMatrix()
        {
            var result = Matrix.CreateZero(NumRows, NumColumns);
            int rowStart = 0;
            for (int i = 0; i < numRowBlocks; ++i)
            {
                int colStart = 0;
                for (int j = 0; j < numColBlocks; ++j)
                {
                    if (blocks[i, j] != null)
                    {
                        result.SetSubmatrix(rowStart, colStart, blocks[i, j]);
                    }
                    colStart += colsPerBlock[j];
                }
                rowStart += rowsPerBlock[i];
            }
            return result;
        }

        public void Multiply(IVectorView lhsVector, IVector rhsVector)
        {
            var input = (BlockVector)lhsVector;
            var output = (BlockVector)rhsVector;
            Preconditions.CheckMultiplicationDimensions(this.numColBlocks, input.Blocks.Length);
            Preconditions.CheckMultiplicationDimensions(this.NumColumns, input.Length);

            Preconditions.CheckSystemSolutionDimensions(this.numRowBlocks, output.Blocks.Length);
            Preconditions.CheckSystemSolutionDimensions(this.NumRows, output.Length);

            output.Clear();
            for (int i = 0; i < numRowBlocks; ++i)
            {
                for (int j = 0; j < numColBlocks; ++j)
                {
                    if (this.blocks[i, j] != null)
                    {
                        output.Blocks[i].AddIntoThis(this.blocks[i, j] * input.Blocks[j]);
                    }
                }
            }
        }

        public IVector Multiply(IVectorView lhsVector) => this * (BlockVector)lhsVector;

        public BlockMatrix Transpose()
        {
            var result = new BlockMatrix(colsPerBlock, rowsPerBlock);
            result.ColMultiplicities = this.RowMultiplicities;
            result.RowMultiplicities = this.ColMultiplicities;
            for (int i = 0; i < numRowBlocks; ++i)
            {
                for (int j = 0; j < numColBlocks; ++j)
                {
                    if (blocks[i, j] != null)
                    {
                        result.AddBlock(j, i, blocks[i, j].Transpose());
                    }
                }
            }
            return result;
        }
    }
}
