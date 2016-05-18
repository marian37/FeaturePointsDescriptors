using System;
using Accord.Math;
using Accord.Math.Decompositions;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace FeaturePointsDescriptors
{
	public class FeaturePointsDescriptors
	{
		private int[,] directions = new int[,] { 
			{ 0, 0, 1 }, { 0, 1, 0 }, { 0, 1, 1 }, { 1, 0, 0 }, { 1, 0, 1 }, 
			{ 1, 1, 0 }, { 1, 1, 1 }, { 0, 0, -1 }, { 0, -1, 0 }, { 0, -1, 1 },
			{ 0, 1, -1 }, { -1, 0, 0 }, { -1, 0, 1 }, { 1, 0, -1 }, { -1, 1, 0 },
			{ 1, -1, 0 }, { 1, 1, -1 }, { 1, -1, 1 }, { -1, 1, 1 }, { 1, -1, -1 }, 
			{ -1, 1, -1 }, { -1, -1, 1 }, { -1, -1, -1 }, { -1, -1, 0 }, { -1, 0, -1 }, { 0, -1, -1 }
		};

		public FeaturePointsDescriptors ()
		{	
		}			

		public List<PointDescriptor> getDescriptors (List<List<List<List<ColorPoint3D>>>> grid, float voxelSize, float maxShellDistance, int numberOfShellBins, int colors, int numberOfPoints)
		{
			List<PointDescriptor> descriptors = new List<PointDescriptor> (numberOfPoints);
			int hueIntervalSize = 360 / colors;
			//int idx = 0;
			int delta = (int)(maxShellDistance / voxelSize);
			float maxShellDistanceSquare = maxShellDistance * maxShellDistance;
			float shellBinSize = maxShellDistance / numberOfShellBins;

			for (int i = 0; i < grid.Count; i++) {
				for (int j = 0; j < grid [i].Count; j++) {
					for (int k = 0; k < grid [i] [j].Count; k++) {
						if (grid [i] [j] [k].Count != 0) {
							ColorPoint3D point = grid [i] [j] [k] [0];
							List<int> descriptor = new List<int> (numberOfShellBins + colors);
							for (int l = 0; l < numberOfShellBins + colors; l++) {
								descriptor.Add (0);
							}
							List<ColorPoint3D> smallNeighbourhood = new List<ColorPoint3D> ();
							List<ColorPoint3D> bigNeighbourhood = new List<ColorPoint3D> ();

							/*
							// BEGIN using variable bin size
							int count = 0;
							float nearestDistance = 0;
							for (int l = 0; l < directions.GetLength (0); l++) {
								if (i + directions [l, 0] >= 0 && i + directions [l, 0] < grid.Count
								    && j + directions [l, 1] >= 0 && j + directions [l, 1] < grid [i].Count
								    && k + directions [l, 2] >= 0 && k + directions [l, 2] < grid [i] [j].Count
								    && grid [i + directions [l, 0]] [j + directions [l, 1]] [k + directions [l, 2]].Count != 0) {
									count++;
									nearestDistance += (float)getDistanceSquare (point, grid [i + directions [l, 0]] [j + directions [l, 1]] [k + directions [l, 2]] [0]);
								}
							}

							if (count == 0) {
								continue;
							}

							nearestDistance /= count;
							float maxShellDistance = nearestDistance * maxShellDistanceCoeff;
							float maxShellDistanceSquare = maxShellDistance * maxShellDistance;
							float shellBinSize = maxShellDistance / numberOfShellBins;
							int delta = (int)(maxShellDistance / voxelSize);

							Console.WriteLine (nearestDistance + " " + count + " " + maxShellDistance);
							// END using variable bin size
							*/

							for (int m = i - delta; m <= i + delta; m++) {
								if (m >= 0 && m < grid.Count) {														
									for (int n = j - delta; n <= j + delta; n++) {
										if (n >= 0 && n < grid [m].Count) {							
											for (int o = k - delta; o <= k + delta; o++) {
												if (o >= 0 && o < grid [m] [n].Count) {
													if (grid [m] [n] [o].Count != 0 && !(i == m && j == n && k == o)) {
														ColorPoint3D neighbor = grid [m] [n] [o] [0];
														float distanceSquare = getDistanceSquare (point, neighbor);
														if (distanceSquare <= maxShellDistanceSquare) {
															bigNeighbourhood.Add (neighbor);

															if (distanceSquare <= maxShellDistanceSquare / 4) {
																smallNeighbourhood.Add (neighbor);
															}

															float distance = (float)Math.Sqrt (distanceSquare);
															int shellBin = (int)(distance / shellBinSize);
															descriptor [shellBin]++;

															float hue = RGBtoHSLHue (neighbor);
															int hueId = (int)(hue / hueIntervalSize);
															descriptor [numberOfShellBins + hueId]++;
														}
													}
												}
											}
										}
									}
								}
							}

							descriptors.Add (new PointDescriptor (point, descriptor, numberOfShellBins, smallNeighbourhood, bigNeighbourhood));
						}
					}
				}
			}

			return descriptors;
		}

		public float getDistanceSquare (ColorPoint3D point, ColorPoint3D neighbor)
		{
			return (point.X - neighbor.X) * (point.X - neighbor.X)
			+ (point.Y - neighbor.Y) * (point.Y - neighbor.Y)
			+ (point.Z - neighbor.Z) * (point.Z - neighbor.Z);
		}

		/*public int getDescriptorSquareDistance (PointDescriptor descriptor1, PointDescriptor descriptor2)
		{
			int distance = 0;
			List<int> intDescriptor1 = descriptor1.IntDescriptor;
			List<int> intDescriptor2 = descriptor2.IntDescriptor;
			for (int i = 0; i < intDescriptor1.Count; i++) {
				distance += (intDescriptor1 [i] - intDescriptor2 [i]) * (intDescriptor1 [i] - intDescriptor2 [i]);
			}
			return distance;
		}*/

		private float RGBtoHSLHue (ColorPoint3D point)
		{
			List<float> rgb = new List<float> (3);
			rgb.Add (point.R / 255f);
			rgb.Add (point.G / 255f);
			rgb.Add (point.B / 255f);

			int maxId = 0;
			int minId = 0;

			for (int i = 1; i < rgb.Count; i++) {
				if (rgb [i] > rgb [maxId]) {
					maxId = i;
				}
				if (rgb [i] < rgb [minId]) {
					minId = i;
				}
			}

			float diff = rgb [maxId] - rgb [minId];		

			if (diff == 0) {
				return 0;
			}

			switch (maxId) {
			case 0: 
				return 60 * (((rgb [1] - rgb [2]) / diff) % 6);
			case 1:
				return 60 * (((rgb [2] - rgb [0]) / diff) + 2);
			case 2: 
				return 60 * (((rgb [0] - rgb [1]) / diff) + 4);
			}

			return 0;
		}

		public static double vectorLength (ColorPoint3D vector)
		{
			return Math.Sqrt (vector.X * vector.X + vector.Y * vector.Y + vector.Z * vector.Z);
		}

		private ColorPoint3D crossProduct (ColorPoint3D u, ColorPoint3D v)
		{
			return new ColorPoint3D (
				u.Y * v.Z - u.Z * v.Y,
				u.Z * v.X - u.X * v.Z,
				u.X * v.Y - u.Y * v.X,
				0, 0, 0
			);
		}

		private bool collinear (List<DescriptorMatch> matches, int[] id, double collinear_treshold, bool first)
		{
			ColorPoint3D a, b, c, x;
			if (first) {
				a = matches [id [0]].Point1.Point;
				b = matches [id [1]].Point1.Point;
				c = matches [id [2]].Point1.Point;
				x = matches [id [3]].Point1.Point;
			} else {
				a = matches [id [0]].Point2.Point;
				b = matches [id [1]].Point2.Point;
				c = matches [id [2]].Point2.Point;
				x = matches [id [3]].Point2.Point;
			}

			ColorPoint3D u = new ColorPoint3D (b.X - a.X, b.Y - a.Y, b.Z - a.Z, 0, 0, 0);
			ColorPoint3D v = new ColorPoint3D (c.X - a.X, c.Y - a.Y, c.Z - a.Z, 0, 0, 0);

			ColorPoint3D n = crossProduct (u, v);

			double d = -(n.X * a.X + n.Y * a.Y + n.Z * a.Z);
			double nLength = vectorLength (n);

			double distance = Math.Abs (n.X * x.X + n.Y * x.Y + n.Z * x.Z + d) / nLength;

			return (distance < collinear_treshold);
		}

		public double[,] ransacTransformation (List<DescriptorMatch> matches, int repeat, float tresholdSquare, double collinear_treshold, int ransacPoints, int numberOfPoints1, int numberOfPoints2)
		{
			Object lockObject = new Object ();

			Random random = new Random ();
			double minError = double.MaxValue;
			int maxInliers = 0;
			double[,] bestCoefficients = null;

			Parallel.For (0, repeat, i => {
				//for (int i = 0; i < repeat; i++) {
				// Choose points
				List<ColorPoint3D> points = new List<ColorPoint3D> ();
				List<ColorPoint3D> imagePoints = new List<ColorPoint3D> ();
				bool[] used1 = new bool[numberOfPoints1];
				bool[] used2 = new bool[numberOfPoints2];
				int[] id = new int[ransacPoints];
				for (int j = 0; j < ransacPoints; j++) {
					while (true) {
						id [j] = random.Next (matches.Count);
						if (!used1 [matches [id [j]].Point1.Point.Idx] && !used2 [matches [id [j]].Point2.Point.Idx]) {
							if (j < ransacPoints - 1 || (!collinear (matches, id, collinear_treshold, true) && !collinear (matches, id, collinear_treshold, false))) {
								used1 [matches [id [j]].Point1.Point.Idx] = true;
								used2 [matches [id [j]].Point2.Point.Idx] = true;
								break;
							}
						}
					}
					points.Add (matches [id [j]].Point2.Point);
					imagePoints.Add (matches [id [j]].Point1.Point);
				}					
					
				double[,] matrix = new double[3 * points.Count, 12];
				double[] rightSide = new double[3 * points.Count];

				for (int j = 0; j < points.Count; j++) {
					for (int k = 0; k < 3; k++) {
						matrix [3 * j + k, 4 * k] = points [j].X;
						matrix [3 * j + k, 4 * k + 1] = points [j].Y;
						matrix [3 * j + k, 4 * k + 2] = points [j].Z;
						matrix [3 * j + k, 4 * k + 3] = 1;
					}
					rightSide [3 * j] = imagePoints [j].X;
					rightSide [3 * j + 1] = imagePoints [j].Y;
					rightSide [3 * j + 2] = imagePoints [j].Z;
				}
					
				double[] coefficientsVector = Matrix.Solve (matrix, rightSide, true);
				//QrDecomposition decomposition = new QrDecomposition (matrix);
				//double[] coefficientsVector = decomposition.Solve (rightSide);

				double[,] coefficients = new double[3, 4];
				for (int j = 0; j < coefficients.GetLength (0); j++) {
					for (int k = 0; k < coefficients.GetLength (1); k++) {
						coefficients [j, k] = coefficientsVector [j * coefficients.GetLength (1) + k];
					}
				}			

				bool[] usedInliers = new bool[numberOfPoints1];
				int inliers = 0;
				double error = 0;
				for (int j = 0; j < matches.Count; j++) {
					if (usedInliers [matches [j].Point1.Point.Idx]) {					
						continue;
					}
					ColorPoint3D estimate = transform (matches [j].Point2.Point, coefficients);
					float distance = getDistanceSquare (matches [j].Point1.Point, estimate);
					error += Math.Sqrt (distance);
					if (distance < tresholdSquare) {
						usedInliers [matches [j].Point1.Point.Idx] = true;
						inliers++;
					}
				}

				lock (lockObject) {
					if (inliers == maxInliers) {
						// check condition
						if (error < minError) {
							minError = error;
							bestCoefficients = coefficients;
						}
					}

					if (inliers > maxInliers) {
						maxInliers = inliers;
						minError = error;
						bestCoefficients = coefficients;
					}
				}
				//}
			});
				
			Console.WriteLine ("Inliers: " + maxInliers);

			return bestCoefficients;
		}

		private ColorPoint3D transform (ColorPoint3D point, double[,] coefficients)
		{
			return new ColorPoint3D ((float)(coefficients [0, 0] * point.X + coefficients [0, 1] * point.Y + coefficients [0, 2] * point.Z + coefficients [0, 3]),
				(float)(coefficients [1, 0] * point.X + coefficients [1, 1] * point.Y + coefficients [1, 2] * point.Z + coefficients [1, 3]),
				(float)(coefficients [2, 0] * point.X + coefficients [2, 1] * point.Y + coefficients [2, 2] * point.Z + coefficients [2, 3]),
				point.R, point.G, point.B);
		}

		public List<ColorPoint3D> transformPointCloud (List<ColorPoint3D> points, double[,] coefficients)
		{
			List<ColorPoint3D> output = new List<ColorPoint3D> (points.Count);

			foreach (ColorPoint3D point in points) {
				output.Add (transform (point, coefficients));
			}

			return output;
		}
	}
}

