#include <iostream>
#include <libgen.h>
#include <string>
#include <unistd.h>

#include <glut.h>

#include <XodrBuilder/XodrBuilder.h>

// https://graphics.stanford.edu/courses/cs248-00/helpsession/opengl/code_example.html#callback_display

// This example visualizes the XODR road
// Needed libraries: opengl, glu, glut

using namespace std;

void InitGraphics();
void display();
void reshape(GLint width, GLint height);
void RenderObjects();

uint listRoad, listBounadries, listCenterlines;
XodrBuilder * xodrBuilder;

int main(int argc, char ** argv)
{
    string xodrFileName = "Town01.zip";
    if (access(xodrFileName.c_str(), R_OK))
    {
        cerr << "No file found: " << xodrFileName << ". Quitting loading XODR." << endl;
        return 1;
    }
    string xodrFileDir = xodrFileName; // need to make copy since dirname corrupts the original string container
    string command = string(string("unzip -o ") + xodrFileName + " -d " + dirname((char*)(xodrFileDir.c_str())));
    auto ret = system(command.c_str());
    if (ret)
    {
        cerr << "Failed unpacking the ZIP file: " << xodrFileName << " with the error code " << ret << endl;
        return 1;
    }
    xodrFileName = xodrFileName.substr(0, xodrFileName.size()-3) + "xodr";

    xodrBuilder = new XodrBuilder(xodrFileName, 1);

    // GLUT Window Initialization:
    glutInit (&argc, argv);
    glutInitWindowSize (800, 600);
    glutInitDisplayMode ( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutCreateWindow ("CS248 GLUT example");
    // Initialize OpenGL graphics state
    InitGraphics();
    glutDisplayFunc (display);
    glutReshapeFunc (reshape);

    glutMainLoop ();

    glDeleteLists(listRoad, 1);
    glDeleteLists(listBounadries, 1);
    glDeleteLists(listCenterlines, 1);

    return 0;
}

void InitGraphics(void)
{
    glDisable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glShadeModel(GL_SMOOTH);

    // roads:
    listRoad = glGenLists(1);
    glNewList(listRoad, GL_COMPILE);
    for (auto && r : xodrBuilder->getBoundaries())
    {
        for (auto && g : r.second)
        {
            for (auto && s : g.second)
            {
                for (auto it1 = s.second.begin(); it1 != s.second.end(); ++it1)
                {
                    auto it2 = it1;
                    advance(it2, 1);
                    if (it2 == s.second.end()) break;

                    glBegin(GL_TRIANGLE_STRIP);
                    for (size_t i = 0; i < min(it1->second.size(), it2->second.size()); ++i)
                    {
                        glVertex3f(it1->second[i].x(), it1->second[i].y(), it1->second[i].z());
                        glVertex3f(it2->second[i].x(), it2->second[i].y(), it2->second[i].z());
                    }
                    glEnd();
                }
            }
        }
    }
    glEndList();

    // draw boundaries:
    listBounadries = glGenLists(1);
    glNewList(listBounadries, GL_COMPILE);
    glPushMatrix();
    glTranslatef(0,0,0.1f);

    for (auto && r : xodrBuilder->getBoundaries())
    {
        for (auto && g : r.second)
        {
            for (auto && s : g.second)
            {
                for (auto && l : s.second)
                {
                    glBegin(GL_LINE_STRIP);
                    for (auto && p : l.second)
                    {
                        glVertex3f(p.x(), p.y(), p.z());
                    }
                    glEnd();
                }
            }
        }
    }
    glEndList();

    // draw centerlines:
    listCenterlines = glGenLists(1);
    glNewList(listCenterlines, GL_COMPILE);
    for (auto && r : xodrBuilder->getCenters())
    {
        for (auto && g : r.second)
        {
            for (auto && s : g.second)
            {
                for (auto && l : s.second)
                {
                    glBegin(GL_POINTS);
                    for (auto && p : l.second)
                    {
                        glVertex3f(p.x(), p.y(), p.z());
                    }
                    glEnd();
                }
            }
        }
    }
    glPopMatrix();

    glEndList();
}

void display()
{
   // Clear frame buffer and depth buffer
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   // Set up viewing transformation, looking down -Z axis
   glLoadIdentity();
   gluLookAt(200, -100, 300, 200, -100, -1, 0, 1, 0);
   // Render the scene
   RenderObjects();
   // Make sure changes appear onscreen
   glutSwapBuffers();
}

void reshape(GLint width, GLint height)
{
   glViewport(0, 0, width, height);
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   gluPerspective(65.0, (float)width / height, 1, 1000);
   glMatrixMode(GL_MODELVIEW);
}

void RenderObjects()
{
    glLineWidth(2);
    glPointSize(1);
    glColor3f(0.5f, 0.5f, 0.5f);
    glCallList(listRoad);
    glColor3f(1.0f, 1.0f, 1.0f);
    glCallList(listBounadries);
    glColor3f(0.75,0.75,1);
    glCallList(listCenterlines);
}