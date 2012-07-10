package org.lmnsolutions.wps;

import java.awt.geom.Point2D;
import java.awt.image.BufferedImage;
import java.awt.image.RenderedImage;
import java.awt.image.WritableRaster;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Map;
import java.util.PriorityQueue;

import javax.measure.quantity.Duration;
import javax.measure.quantity.Length;
import javax.measure.quantity.Velocity;
import javax.measure.unit.NonSI;
import javax.measure.unit.SI;
import javax.measure.unit.Unit;

import org.geoserver.wps.gs.GeoServerProcess;
import org.geoserver.wps.gs.PolygonExtractionProcess;
import org.geoserver.wps.jts.DescribeParameter;
import org.geoserver.wps.jts.DescribeProcess;
import org.geoserver.wps.jts.DescribeResult;
import org.geotools.coverage.grid.GridCoordinates2D;
import org.geotools.coverage.grid.GridCoverage2D;
import org.geotools.coverage.grid.GridCoverageFactory;
import org.geotools.data.simple.SimpleFeatureCollection;
import org.geotools.geometry.DirectPosition2D;
import org.geotools.process.ProcessException;
import org.geotools.referencing.GeodeticCalculator;
import org.geotools.util.NullProgressListener;
import org.opengis.coverage.grid.GridGeometry;
import org.opengis.geometry.DirectPosition;
import org.opengis.geometry.MismatchedDimensionException;
import org.opengis.referencing.operation.MathTransform;
import org.opengis.referencing.operation.TransformException;

import com.vividsolutions.jts.geom.Geometry;

@DescribeProcess(title = "HelicopterReachability", description = "Given a raster, find the area a helicopter can fly in.")
public class HelicopterReachability implements GeoServerProcess {

    @DescribeResult(name = "result", description = "The collection of result polygons.")
    public SimpleFeatureCollection execute(
            @DescribeParameter(name = "raster", description = "The raster to check against.") GridCoverage2D coverage,
    		@DescribeParameter(name = "startPoint", description = "The point to start at.") Geometry startPoint,
    		@DescribeParameter(name = "maxElevation", description = "Maximum Elevation that can be considered passable in meters") double maxElevation,
    		@DescribeParameter(name = "airSpeed", description = "vehicle air speed in knots") double airSpeed,
    		@DescribeParameter(name = "time", description = "time availible to reach a destination in minutes") double time)
            throws Exception {
    	

        if (coverage == null) {
            throw new ProcessException("Invalid input, source grid coverage should be not null");
        }
    	
    	return FloodFill.floodFill(coverage, startPoint, maxElevation, time /*use Duration instead*/, airSpeed);
    }
}



class PlannerNode {
	int x = -1;
	int y = -1;
	double g = 0;
	PlannerNode parent = null;

	public PlannerNode(PlannerNode parent, int x, int y, double g) {
		this.parent = parent;
		this.x = x;
		this.y = y;
		this.g = g;
	}

	public int getHash() {
		return (this.x << 16 | this.y);
	}

	public String toString() {
		return ("x: " + this.x + ", y: " + this.y + ", g: " + this.g + ", parent: " + (this.parent != null ? true : false) + ", hash: " + this.getHash());
	}
}

class PlannerNodeComparator implements Comparator<PlannerNode> {
	@Override
	public int compare(PlannerNode a, PlannerNode b) {
		if (a.g < b.g) {
			return -1;
		}
		if (a.g > b.g) {
			return 1;
		}
		return 0;
	}
}

class FloodFill {
	
	protected static GridCoordinates2D coord = new GridCoordinates2D(0,0);
	protected static int[] intArrayOne = new int[1];
    
    public static double getElevation(GridCoverage2D coverage, int x, int y) {
		coord.setCoordinateValue(0, x);
		coord.setCoordinateValue(1, y); 		
		coverage.evaluate(coord, intArrayOne);
    	return intArrayOne[0];
    }
    
	private static DirectPosition pixelToMap(DirectPosition2D pixel, GridGeometry grid) throws MismatchedDimensionException, TransformException
	{
		MathTransform xform = grid.getGridToCRS();
		return xform.transform(pixel, null);
	}
	
	private static DirectPosition mapToPixel(DirectPosition2D coord, GridGeometry grid) throws MismatchedDimensionException, TransformException
	{
		MathTransform xform = grid.getGridToCRS();
		xform = xform.inverse();
		return xform.transform(coord, null);
	}
	
	private static double nauticalMilesTraveled(double knots, double minutes)
	{
		return knots * (minutes / 60);
	}
	private static Unit<Length> nauticalMilesTraveled(Unit<Velocity> knots, Unit<Duration> duration)
	{
		return knots.times(duration).asType(Length.class);
	}
	
	private static double nauticalMilesToMeters(double nm)
	{
		return nm * 1852;
	}
	
	private static Unit<Length> nauticalMilesToMeters(Unit<Length> nm)
	{
		return nm.transform(NonSI.NAUTICAL_MILE.getConverterTo(SI.METER));
	}
	
	private static Unit<Length> geodeticDistanceInMeters(Point2D source, Point2D target)
	{
		GeodeticCalculator calculator = new GeodeticCalculator();
		calculator.setStartingGeographicPoint(source);
		calculator.setDestinationGeographicPoint(target);
		return SI.METER.times(calculator.getOrthodromicDistance());
	}
    
	public static SimpleFeatureCollection floodFill(GridCoverage2D coverage, Geometry point, double maxElevation, double time /*use Duration instead*/, double airSpeed) throws Exception{

		/*
		Unit<Length> dist_traveled = nauticalMilesTraveled(NonSI.KNOT.times(airSpeed), NonSI.MINUTE.times(time));
		System.out.println("nautical miles traveled");		
		System.out.println(dist_traveled);

		System.out.println("nautical miles traveled in meters (1 nm = 1,852 meters)");
		System.out.println(nauticalMilesToMeters(dist_traveled));
		*/

		// Max distance we can travel in meters given the airspeed and time limitations.
		double gMax = nauticalMilesToMeters(nauticalMilesTraveled(airSpeed, time));
		System.out.println("gmax: " + gMax);

		RenderedImage image = coverage.getRenderedImage();
		
		int width = image.getWidth();
		int height = image.getHeight();
		
		DirectPosition pointIndex = mapToPixel(new DirectPosition2D(point.getCoordinate().x, point.getCoordinate().y), coverage.getGridGeometry());
		
		int pointIndexX = (int)pointIndex.getOrdinate(0);
		int pointIndexY = (int)pointIndex.getOrdinate(1);
		
//		DirectPosition pp = pixelToMap(pointIndexX, pointIndexY, coverage.getGridGeometry());
		
		BufferedImage image2 = new BufferedImage(width,height,BufferedImage.TYPE_BYTE_BINARY);
		WritableRaster raster2 = image2.getRaster();

		double elev = getElevation(coverage, pointIndexX, pointIndexY);
		
		if (elev >= maxElevation) {
			return null;
		}
		
		Comparator<PlannerNode> comparator = new PlannerNodeComparator();
        PriorityQueue<PlannerNode> openQueue = new PriorityQueue<PlannerNode>(10, comparator);
        
        Map<Integer,PlannerNode> allMap = new HashMap<Integer, PlannerNode>();
        
        PlannerNode startNode =  new PlannerNode(null, pointIndexX, pointIndexY, 0);
        
        openQueue.add(startNode);
        allMap.put(startNode.getHash(), startNode);

        while (openQueue.size() != 0){
        	
        	PlannerNode currentNode = openQueue.poll();
        	
        	for (int ii = 0; ii < 3; ii++) {
        		for (int j = 0; j < 3; j++) {
        			// skip center spot
        			if (ii == 1 && j == 1)
        				continue;
        			
        			int cx = currentNode.x - 1 + ii;
        			int cy = currentNode.y - 1 + j;
        			
        			// if we are in bounds
        			if (cx > -1 && cx < width && cy > -1 && cy < height) {
        				
        				double elevation = getElevation(coverage, cx, cy);
        				
        				if (elevation < maxElevation) {
	        				
	        				int hash = (cx << 16 | cy);
	        				
	        				PlannerNode existingNode = allMap.get(hash);
//	        				print("-- child.x: ", cx, ", child.y: ", cy, ", hash: ", hash, ", new: ", (existingNode==undefined?true:false));
	        				
	        				// only if this spot has not been visited in the past
	        				if (existingNode == null) {
	        					double cg = 0;
	        					
	        					if (cx == currentNode.x || cy == currentNode.y) {
	        						cg = currentNode.g + 1;
	        					} else {
	        						cg = currentNode.g + 1.41421356;
	        					}
	        					
//	        					System.out.println("-- child.x: "+ cx + ", child.y: " + cy + ", cg: " + cg + ", hash: " + hash + ", new: " + (existingNode==null?true:false));
	        				
	        					// do not push nodes that are beyond reachability cut-off
	        					if(cg <= gMax) {
	    							PlannerNode childNode =  new PlannerNode(currentNode, cx, cy, cg);
	    							openQueue.add(childNode);
	    							// added to allMap so we do not regenerate this node again 
	    							allMap.put(hash, childNode);
	    							//put a one where this child is in the output raster 
	    							intArrayOne[0] = 1;
	    							raster2.setPixel(cx, cy, intArrayOne);
	        					}
	        				} else {
	        					//print("-- location is already closed: " + closedNode.toString())
	        				}
        				}
        			}
        		}
        	}	
         }   
        
        
        GridCoverageFactory gcf = new GridCoverageFactory();
        GridCoverage2D gc = gcf.create("name", image2, coverage.getEnvelope());
          
    	final PolygonExtractionProcess process = new PolygonExtractionProcess();
		final SimpleFeatureCollection fc = process.execute(
				gc, 
				0,
				true, 
				null,
				null,
				new NullProgressListener());
		
/*
		SimpleFeatureIterator fi = fc.features();

		while (fi.hasNext()) {
			SimpleFeature sf = fi.next();
			Double value = (Double) sf.getAttribute("value");
		}
		fi.close();
*/		
        return fc;
	}
}