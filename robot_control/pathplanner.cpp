#include "pathplanner.h"

using namespace std;
using namespace cv;

// Sequences used for checking surrounding pixels (clockwise)
// Note: Direction on map is flipped due to the nature of matrices.
const int x_seq[8] = {0,1,1,1,0,-1,-1,-1};
const int y_seq[8] = {1,1,0,-1,-1,-1,0,1};


// Universal Colors
const Vec3b color_black = {0,0,0};
const Vec3b color_white = {255,255,255};
const Vec3b color_red = {0,0,255};

// Path colors
#define INDEX_MISC 0
#define INDEX_PATH 1
#define INDEX_BRUSHFIRE 2
#define COLORVAL_PATH 20
#define COLORVAL_MISC 21
#define COLORVAL_MAX 255
#define COLORVAL_MIN 0

Pathplanner::Pathplanner()
{

}

Pathplanner::Pathplanner(Mat _image)
{
    // Imported image.
    image = _image;

    // Clone image for brushfire algorithm.
    image_brushfire = _image.clone();

    // Clone image for path drawing.
    image_path = _image.clone();

    // Security variable.
    isCalculated = false;
}

Mat Pathplanner::getPathImage()
{
    if(isCalculated)
    {
        return image_path;
    }
    else
    {
        cout << "Pathplanner:\tPath not yet calculated. ";
        cout << "Please perform 'calculatePath()'." << endl;
        return image;
    }
}

vector<Point> Pathplanner::getEndPoints()
{
    if(isCalculated)
    {
        return endPoints;
    }
    else
    {
        cout << "Pathplanner:\tPath not yet calculated. ";
        cout << "Please perform 'calculatePath()'." << endl;
        return endPoints;
    }
}

vector<Point> Pathplanner::getPathPoints()
{
    if(isCalculated)
    {
        return pathPoints;
    }
    else
    {
        cout << "Pathplanner:\tPath not yet calculated. ";
        cout << "Please perform 'calculatePath()'." << endl;
        return pathPoints;
    }
}

vector<Point> Pathplanner::getPath(Point src, Point dst)
{

    // Establish test variable.
    bool abort = false;

    // Test if source is located on path.
    if((image_path.at<Vec3b>(src)[INDEX_PATH] != COLORVAL_PATH))
    {
        cout << "Pathplanner:\tSource " << src << " not part of path." << endl;
        abort = true;
    }

    // Test if destination is located on path.
    if((image_path.at<Vec3b>(dst)[INDEX_PATH] != COLORVAL_PATH))
    {
        cout << "Pathplanner:\tDestination " << dst << " not part of path." << endl;
        abort = true;
    }

    // If either point is not on path, abort.
    if(abort)
    {
        cout << "Pathplanner:\tAborting..." << endl;
        return pathPoints;
    }

    // Run recursive simplified A* algorithm.
    pathFinder(src, 1);

    // Create vector for storing path points.
    vector<Point> myPath;

    // Initialize pathfinding at destination.
    Point current_point = dst;
    Point tmp_point = dst;

    // While source is not reached...
    while(current_point != src)
    //for(int i = 0; i< 200; i++)
    {
        // Store current point.
        myPath.insert(myPath.begin(),current_point);

        // Initialize test point.
        tmp_point = current_point;

        // For all surrounding pixels...
        for(int i = 0; i < 8; i++)
        {
            // If A* value is smaller than smallest surrounding value yet...
            if(image_path.at<Vec3b>(Point(current_point.x+x_seq[i], current_point.y+y_seq[i]))[INDEX_BRUSHFIRE]
                    < ((image_path.at<Vec3b>(current_point)[INDEX_BRUSHFIRE])))
            {
                // Save that value as test point.
                tmp_point = Point(current_point.x+x_seq[i], current_point.y+y_seq[i]);
            }
        }

        // In case of no point is found (bad apple)
        if(tmp_point == current_point)
        {
            cout << "Pathplanner:\tDistance error! " << image_path.at<Vec3b>(current_point) << endl;

            // For all surrounding pixels...
            for(int i = 0; i < 8; i++)
            {
                // Create fix here if problem is ever encountered.
                //cout << image_path.at<Vec3b>(Point(current_point.x+x_seq[i], current_point.y+y_seq[i])) << endl;
            }
        }

        // Set current point so smallest surrounding point.
        current_point = tmp_point;
    }


    // Clean path of A* values.
    for(uint i = 0; i<pathPoints.size(); i++)
    {
        image_path.at<Vec3b>(pathPoints[i])[INDEX_BRUSHFIRE] = COLORVAL_MAX;
    }

    // Return resulting array of path points.
    return myPath;
}

void Pathplanner::pathFinder(Point point, int distance)
{

    if(image_path.at<Vec3b>(point)[INDEX_BRUSHFIRE]<distance)
    {
        return;
    }

    image_path.at<Vec3b>(point)[INDEX_BRUSHFIRE] = distance;

    for(int i = 0; i<8; i++)
    {
        if(image_path.at<Vec3b>(Point(point.x+x_seq[i], point.y+y_seq[i]))[INDEX_PATH] == COLORVAL_PATH)
        {
            pathFinder(Point(point.x+x_seq[i], point.y+y_seq[i]), distance+1);
        }
    }
}

void Pathplanner::calculatePath()
{
    if(isCalculated)
    {
        cout << "Pathplanner:\tPath already calculated, aborting..." << endl;
        return;
    }

    brushfire();
    cout << "Pathplanner:\tBrushfire complete..." << endl;
    pathLocalMaxima();
    cout << "Pathplanner:\tLocal maxima found..." << endl;
    pathPreClean();
    cout << "Pathplanner:\tFirst path cleaning complete..." << endl;
    pathConnect();
    cout << "Pathplanner:\tPath routing complete..." << endl;
    pathPostClean();
    cout << "Pathplanner:\tSecond path cleaning complete..." << endl;
    storePoints();
    cout << "Pathplanner:\tPathpoints and endpoints stored..." << endl;

    isCalculated = true;
    cout << "Pathplanner:\tPath calculated successfully!" << endl;
}

bool Pathplanner::hasWhite()
{
    for (int rows = 0; rows < image_brushfire.rows; rows++)
    {
        for (int cols = 0; cols < image_brushfire.cols; cols++)
        {
            if(image_brushfire.at<Vec3b>(Point(cols,rows)) == color_white)
            {
                return true;
            }
        }
    }
    return false;
}

void Pathplanner::brushfire()
{
    int iteration = 0;

    // Perform brushfire
    while(hasWhite())
    {
        for (int rows = 0; rows < image_brushfire.rows; rows++)
        {
            for (int cols = 0; cols < image_brushfire.cols; cols++)
            {
                if(image_brushfire.at<Vec3b>(Point(cols,rows)) == color_white)
                {
                    for(int i = 0; i<8;i++)
                    {
                        if((image_brushfire.at<Vec3b>(Point((cols+x_seq[i]),(rows+y_seq[i])))[INDEX_BRUSHFIRE] == iteration))
                        {
                            image_brushfire.at<Vec3b>(Point(cols,rows))[INDEX_BRUSHFIRE] = iteration+1;
                        }
                    }
                }
            }
        }
        iteration++;
    }
}

void Pathplanner::pathLocalMaxima()
{
    bool smallest = true;

    //Find initial route (Highlight largest value)
    for (int rows = 5; rows < image.rows-5; rows++)
    {
        for (int cols = 5; cols < image.cols-5; cols++)
        {
            if(image_brushfire.at<Vec3b>(Point(cols,rows)) != color_black)
            {
                smallest = true;

                for(int i = 0; i<8;i++)
                {
                    if((image_brushfire.at<Vec3b>(Point((cols+x_seq[i]),(rows+y_seq[i])))[INDEX_BRUSHFIRE]
                        > image_brushfire.at<Vec3b>(Point(cols,rows))[INDEX_BRUSHFIRE]))
                    {
                        smallest = false;
                    }
                }

                if(smallest)
                {
                    image_path.at<Vec3b>(Point(cols,rows))[INDEX_PATH] = COLORVAL_PATH;
                }
            }
        }
    }
}

void Pathplanner::pathPreClean()
{
    int hor_seq[8] = {COLORVAL_MAX,COLORVAL_MAX,COLORVAL_PATH,COLORVAL_PATH,COLORVAL_PATH,COLORVAL_PATH,COLORVAL_PATH,COLORVAL_MAX};
    int ver_seq[8] = {COLORVAL_PATH,COLORVAL_PATH,COLORVAL_PATH,COLORVAL_PATH,COLORVAL_PATH,COLORVAL_MAX,COLORVAL_MAX,COLORVAL_MAX};
    int hor_delSeq[8] = {COLORVAL_MAX,COLORVAL_MAX,COLORVAL_MISC,COLORVAL_MAX,COLORVAL_MAX,COLORVAL_MAX,COLORVAL_MISC,COLORVAL_MAX};
    int ver_delSeq[8] = {COLORVAL_MISC,COLORVAL_MAX,COLORVAL_MAX,COLORVAL_MAX,COLORVAL_MISC,COLORVAL_MAX,COLORVAL_MAX,COLORVAL_MAX};

    int pixelPath = 0;
    int pixelBlack = 0;

    // Remove single pixels
    for (int rows = 0; rows < image.rows; rows++)
    {
        for (int cols = 0; cols < image.cols; cols++)
        {
            if(image_path.at<Vec3b>(Point(cols,rows))[INDEX_PATH] == COLORVAL_PATH)
            {
                pixelPath = 0;
                pixelBlack = 0;
                for(int i = 0; i<8;i++)
                {
                    if(image_path.at<Vec3b>(Point((cols+x_seq[i]),(rows+y_seq[i])))[INDEX_PATH] == COLORVAL_PATH)
                    {
                        pixelPath++;
                    }
                    if(image.at<Vec3b>(Point((cols+x_seq[i]),(rows+y_seq[i]))) == color_black)
                    {
                        pixelBlack++;
                    }
                }

                if((pixelPath == 0)||(pixelBlack != 0))
                {
                    image_path.at<Vec3b>(Point(cols,rows))[INDEX_PATH] = COLORVAL_MAX;
                }
            }
        }
    }

    // Mark double pixels
    pixelPath = 0;

    for (int rows = 0; rows < image.rows; rows++)
    {
        for (int cols = 0; cols < image.cols; cols++)
        {
            if(image_path.at<Vec3b>(Point(cols,rows))[INDEX_PATH] == COLORVAL_PATH)
            {
                pixelPath = 0;
                for(int i = 0; i<8;i++)
                {
                    if(image_path.at<Vec3b>(Point((cols+x_seq[i]),(rows+y_seq[i])))[INDEX_PATH] == COLORVAL_PATH)
                    {
                        pixelPath++;
                    }
                }

                if((pixelPath == 1))
                {
                    image_path.at<Vec3b>(Point(cols,rows))[INDEX_MISC] = COLORVAL_MISC;
                }
            }
        }
    }

    // Flush double pixels
    for (int rows = 0; rows < image.rows; rows++)
    {
        for (int cols = 0; cols < image.cols; cols++)
        {
            if(image_path.at<Vec3b>(Point(cols,rows))[INDEX_MISC] == COLORVAL_MISC)
            {
                for(int i = 0; i<8;i++)
                {
                    if(image_path.at<Vec3b>(Point((cols+x_seq[i]),(rows+y_seq[i])))[INDEX_MISC] == COLORVAL_MISC)
                    {
                        image_path.at<Vec3b>(Point(cols,rows))[INDEX_PATH] = COLORVAL_MAX;
                        image_path.at<Vec3b>(Point(cols,rows))[INDEX_MISC] = COLORVAL_MAX;
                        image_path.at<Vec3b>(Point((cols+x_seq[i]),(rows+y_seq[i])))[INDEX_PATH] = COLORVAL_MAX;
                        image_path.at<Vec3b>(Point((cols+x_seq[i]),(rows+y_seq[i])))[INDEX_MISC] = COLORVAL_MAX;
                    }
                }

                image_path.at<Vec3b>(Point(cols,rows))[INDEX_MISC] = COLORVAL_MAX;
            }
        }
    }

    //Mark 2 row paths
    bool horPattern = true;
    bool verPattern = true;

    for (int rows = 0; rows < image.rows; rows++)
    {
        for (int cols = 0; cols < image.cols; cols++)
        {
            horPattern = true;
            verPattern = true;

            for(int i = 0; i<8;i++)
            {
                if(image_path.at<Vec3b>(Point((cols+x_seq[i]),(rows+y_seq[i])))[INDEX_PATH] != hor_seq[i])
                {
                    horPattern = false;
                }

                if(image_path.at<Vec3b>(Point((cols+x_seq[i]),(rows+y_seq[i])))[INDEX_PATH] != ver_seq[i])
                {
                    verPattern = false;
                }
            }

            if(horPattern)
            {
                image_path.at<Vec3b>(Point(cols,rows))[INDEX_MISC] = COLORVAL_MISC;

                for(int i = 0; i<8; i++)
                {
                    image_path.at<Vec3b>(Point((cols+x_seq[i]),(rows+y_seq[i])))[INDEX_MISC] = hor_delSeq[i];
                }
            }

            if(verPattern)
            {
                image_path.at<Vec3b>(Point(cols,rows))[INDEX_MISC] = COLORVAL_MISC;

                for(int i = 0; i<8; i++)
                {
                    image_path.at<Vec3b>(Point((cols+x_seq[i]),(rows+y_seq[i])))[INDEX_MISC] = ver_delSeq[i];
                }
            }
        }
    }


    // Flush 2 row paths
    for (int rows = 0; rows < image.rows; rows++)
    {
        for (int cols = 0; cols < image.cols; cols++)
        {
            if(image_path.at<Vec3b>(Point(cols,rows))[INDEX_MISC]==COLORVAL_MISC)
            {
                image_path.at<Vec3b>(Point(cols,rows))[INDEX_MISC] = COLORVAL_MAX;
                image_path.at<Vec3b>(Point(cols,rows))[INDEX_PATH] = COLORVAL_MAX;
            }
        }
    }

}

void Pathplanner::pathConnect()
{
    int pixelPath = 0;
    int seqIndex = 0;
    int seqOffset = 0;
    int seqIndexNew = 0;
    int pathLargestVal = 0;
    int pathCurrentVal = 0;
    Point pointCurrent;
    Point pointNext;
    bool foundSame = false;
    Mat image_path_previous = image_path.clone();

    while(true)
    {
        image_path_previous = image_path.clone();

        for (int rows = 0; rows < image.rows; rows++)
        {
            for (int cols = 0; cols < image.cols; cols++)
            {
                if(image_path.at<Vec3b>(Point(cols,rows))[INDEX_PATH] == COLORVAL_PATH)
                {
                    pixelPath = 0;
                    seqIndex = 0;
                    seqOffset = 0;
                    seqIndexNew = 0;
                    pathLargestVal = 0;
                    pathCurrentVal = 0;
                    pointCurrent= Point(cols,rows);
                    pointNext = Point(cols,rows);
                    foundSame = false;

                    for(int i = 0; i<8; i++)
                    {
                        if(image_path.at<Vec3b>(Point((cols+x_seq[i]),(rows+y_seq[i])))[INDEX_PATH] == COLORVAL_PATH)
                        {
                            pixelPath++;
                            seqIndex = i;
                        }
                    }

                    if(pixelPath == 1)
                    {
                        seqOffset = seqIndex+4;
                        pathLargestVal = image_brushfire.at<Vec3b>(Point(cols,rows))[2];
                        pointNext = Point(cols,rows);

                        for(int i=0; i<8; i++)
                        {
                            seqIndexNew = (i+seqOffset)%8;

                            pointCurrent = Point((cols+x_seq[seqIndexNew]),(rows+y_seq[seqIndexNew]));
                            pathCurrentVal = image_brushfire.at<Vec3b>(Point((cols+x_seq[seqIndexNew]),(rows+y_seq[seqIndexNew])))[INDEX_BRUSHFIRE];
                            if((foundSame == false)
                                    && (image_brushfire.at<Vec3b>(pointCurrent)[INDEX_BRUSHFIRE]
                                        >= image_brushfire.at<Vec3b>(pointNext)[INDEX_BRUSHFIRE]))
                            {
                                pathLargestVal = pathCurrentVal;
                                pointNext = Point((cols+x_seq[seqIndexNew]),(rows+y_seq[seqIndexNew]));
                                foundSame = true;
                            }
                            else if(pathCurrentVal > pathLargestVal)
                            {
                                pathLargestVal = pathCurrentVal;
                                pointNext = Point((cols+x_seq[seqIndexNew]),(rows+y_seq[seqIndexNew]));
                            }
                        }

                        image_path.at<Vec3b>(pointNext)[INDEX_PATH] = COLORVAL_PATH;
                    }
                }
            }
        }

        // Magic way of comparing images.
        if(sum(image_path != image_path_previous) == Scalar(0,0,0,0))
        {
            return;
        }
    }
}

void Pathplanner::pathPostClean()
{
    int index1 = 0;
    int index2 = 0;
    int pixelPath = 0;

    for (int rows = 0; rows < image.rows; rows++)
    {
        for (int cols = 0; cols < image.cols; cols++)
        {
            if(image_path.at<Vec3b>(Point(cols,rows))[INDEX_PATH] == COLORVAL_PATH)
            {
                index1 = 0;
                index2 = 0;
                pixelPath = 0;

                for(int i = 0; i<8; i++)
                {
                    if(image_path.at<Vec3b>(Point(cols+x_seq[i],rows+y_seq[i]))[INDEX_PATH]
                            == COLORVAL_PATH)
                    {
                        index2 = index1;
                        index1 = i;
                        pixelPath++;
                    }
                }

                // Remember that the sequence is flipped...
                if(pixelPath == 2)
                {
                    if(((index1 == 2) && (index2 == 1))||((index1 == 4) && (index2 == 3)))
                    {
                        image_path.at<Vec3b>(Point(cols,rows))[INDEX_PATH] = COLORVAL_MAX;
                    }

                }
            }
        }
    }

}

void Pathplanner::storePoints()
{
    int pixelPath = 0;

    for (int rows = 0; rows < image.rows; rows++)
    {
        for (int cols = 0; cols < image.cols; cols++)
        {
            if(image_path.at<Vec3b>(Point(cols,rows))[INDEX_PATH] == COLORVAL_PATH)
            {

                pixelPath = 0;
                pathPoints.push_back(Point(cols,rows));

                for(int i = 0; i<8;i++)
                {
                    if(image_path.at<Vec3b>(Point((cols+x_seq[i]),(rows+y_seq[i])))[INDEX_PATH] == COLORVAL_PATH)
                    {
                        pixelPath++;
                    }
                }

                if((pixelPath == 1))
                {
                    endPoints.push_back(Point(cols,rows));
                }
            }
        }
    }
}
