using System;
using System.Collections.Generic;

namespace FeaturePointsDescriptors
{
	public class VoxelGridFilter
	{

		private List<List<List<List<ColorPoint3D>>>> grid;
		private float maxX;
		private float minX;
		private float maxY;
		private float minY;
		private float maxZ;
		private float minZ;

		public VoxelGridFilter ()
		{
		}

		public List<ColorPoint3D> filter (List<ColorPoint3D> points, float voxelSize)
		{
			List<ColorPoint3D> output = new List<ColorPoint3D> (points.Count);
			prepare ();
			getRange (points);
			dividePoints (points, voxelSize);
			int idx = 0;
			for (int i = 0; i < grid.Count; i++) {
				for (int j = 0; j < grid [i].Count; j++) {
					for (int k = 0; k < grid [i] [j].Count; k++) {
						if (grid [i] [j] [k].Count > 0) {
							if (grid [i] [j] [k].Count == 1) {
								grid [i] [j] [k] [0].Idx = idx++;
								output.Add (grid [i] [j] [k] [0]);
							} else {
								ColorPoint3D center = calculateCenter (grid [i] [j] [k]);
								center.Idx = idx++;
								output.Add (center);
								grid [i] [j] [k] = new List<ColorPoint3D> (1);
								grid [i] [j] [k].Add (center);
							}
						}
					}
				}
			}		
			return output;
		}

		private void prepare ()
		{
			maxX = float.MinValue;
			minX = float.MaxValue;
			maxY = float.MinValue;
			minY = float.MaxValue;
			maxZ = float.MinValue;
			minZ = float.MaxValue;
		}

		private void getRange (List<ColorPoint3D> points)
		{
			foreach (ColorPoint3D point in points) {
				if (point.X > maxX) {
					maxX = point.X;
				} else {
					if (point.X < minX) {
						minX = point.X;
					}
				}

				if (point.Y > maxY) {
					maxY = point.Y;
				} else {
					if (point.Y < minY) {
						minY = point.Y;
					}
				}

				if (point.Z > maxZ) {
					maxZ = point.Z;
				} else {
					if (point.Z < minZ) {
						minZ = point.Z;
					}
				}
			}
		}

		private void dividePoints (List<ColorPoint3D> points, float voxelSize)
		{
			int xCount = (int)Math.Ceiling ((maxX - minX) / voxelSize);
			int yCount = (int)Math.Ceiling ((maxY - minY) / voxelSize);
			int zCount = (int)Math.Ceiling ((maxZ - minZ) / voxelSize);
			grid = new List<List<List<List<ColorPoint3D>>>> (xCount);

			// vytvorenie 4-rozmerného poľa
			for (int i = 0; i < xCount; i++) {
				List<List<List<ColorPoint3D>>> list = new List<List<List<ColorPoint3D>>> (yCount);
				grid.Add (list);
				for (int j = 0; j < yCount; j++) {
					List<List<ColorPoint3D>> list2 = new List<List<ColorPoint3D>> (zCount);
					list.Add (list2);
					for (int k = 0; k < zCount; k++) {
						List<ColorPoint3D> list3 = new List<ColorPoint3D> ();
						list2.Add (list3);
					}
				}
			}

			// samotné zadelenie bodov
			foreach (ColorPoint3D point in points) {
				float x = point.X;
				float y = point.Y;
				float z = point.Z;
				int xID = (int)Math.Floor ((x - minX) / voxelSize);
				int yID = (int)Math.Floor ((y - minY) / voxelSize);
				int zID = (int)Math.Floor ((z - minZ) / voxelSize);
				grid [xID] [yID] [zID].Add (point);
			}
		}

		private ColorPoint3D calculateCenter (List<ColorPoint3D> points)
		{
			float sumX = 0;
			float sumY = 0;
			float sumZ = 0;
			int sumR = 0;
			int sumG = 0;
			int sumB = 0;
			int count = points.Count;

			foreach (ColorPoint3D point in points) {
				sumX += point.X;
				sumY += point.Y;
				sumZ += point.Z;
				sumR += point.R;
				sumG += point.G;
				sumB += point.B;
			}

			return new ColorPoint3D (sumX / count, sumY / count, sumZ / count, sumR / count, sumG / count, sumB / count);
		}

		public List<List<List<List<ColorPoint3D>>>> getGrid {
			get {
				return grid;
			}
		}
	}
}

