// PointCloudViewer.cpp : Defines the entry point for the application.
//

/*
* Example of a Windows OpenGL program.
* The OpenGL code is the same as that used in
* the X Window System sample
*/
#include <windows.h> 
#include <windowsx.h>

#include <GL/gl.h> 
#include <GL/glu.h> 

#include <vector>

#include <iostream>
#include <fstream>
#include <string>

/* Windows globals, defines, and prototypes */
CHAR szAppName[] = "Win OpenGL";
HWND  ghWnd;
HDC   ghDC;
HGLRC ghRC;

#define SWAPBUFFERS SwapBuffers(ghDC) 
#define BLACK_INDEX     0 
#define RED_INDEX       13 
#define GREEN_INDEX     14 
#define BLUE_INDEX      16 
#define WIDTH           300 
#define HEIGHT          200 

LONG WINAPI MainWndProc(HWND, UINT, WPARAM, LPARAM);
BOOL bSetupPixelFormat(HDC);

/* OpenGL globals, defines, and prototypes */
GLfloat latitude, longitude, latinc, longinc;
GLdouble radius;
GLfloat scaleX, scaleY, scaleZ;

#define GLOBE    1 
#define CYLINDER 2 
#define CONE     3 
#define POINT_CLOUD 4

GLvoid resize(GLsizei, GLsizei);
GLvoid initializeGL(GLsizei, GLsizei);
GLvoid drawScene(GLvoid);
void polarView(GLdouble, GLdouble, GLdouble, GLdouble);


int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
    MSG        msg;
    WNDCLASS   wndclass;

    /* Register the frame class */
    wndclass.style = 0;
    wndclass.lpfnWndProc = (WNDPROC)MainWndProc;
    wndclass.cbClsExtra = 0;
    wndclass.cbWndExtra = 0;
    wndclass.hInstance = hInstance;
    wndclass.hIcon = LoadIcon(hInstance, szAppName);
    wndclass.hCursor = LoadCursor(NULL, IDC_ARROW);
    wndclass.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    wndclass.lpszMenuName = szAppName;
    wndclass.lpszClassName = szAppName;

    if (!RegisterClass(&wndclass))
        return FALSE;

    /* Create the frame */
    ghWnd = CreateWindow(szAppName,
        "Generic OpenGL Sample",
        WS_OVERLAPPEDWINDOW | WS_CLIPSIBLINGS | WS_CLIPCHILDREN,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        WIDTH,
        HEIGHT,
        NULL,
        NULL,
        hInstance,
        NULL);

    /* make sure window was created */
    if (!ghWnd)
        return FALSE;

    /* show and update main window */
    ShowWindow(ghWnd, nCmdShow);

    UpdateWindow(ghWnd);

    /* animation loop */
    while (1) {
        /*
        *  Process all pending messages
        */

        while (PeekMessage(&msg, NULL, 0, 0, PM_NOREMOVE) == TRUE)
        {
            if (GetMessage(&msg, NULL, 0, 0))
            {
                TranslateMessage(&msg);
                DispatchMessage(&msg);
            }
            else {
                return TRUE;
            }
        }
        drawScene();
    }
}

/* main window procedure */
LONG WINAPI MainWndProc(
    HWND    hWnd,
    UINT    uMsg,
    WPARAM  wParam,
    LPARAM  lParam)
{
    LONG    lRet = 1;
    PAINTSTRUCT    ps;
    RECT rect;

    int xPos, yPos;
    int fwKeys, zDelta, mult;

    switch (uMsg) {

    case WM_CREATE:
        ghDC = GetDC(hWnd);
        if (!bSetupPixelFormat(ghDC))
            PostQuitMessage(0);

        ghRC = wglCreateContext(ghDC);
        wglMakeCurrent(ghDC, ghRC);
        GetClientRect(hWnd, &rect);
        initializeGL(rect.right, rect.bottom);
        break;

    case WM_PAINT:
        BeginPaint(hWnd, &ps);
        EndPaint(hWnd, &ps);
        break;

    case WM_SIZE:
        GetClientRect(hWnd, &rect);
        resize(rect.right, rect.bottom);
        break;

    case WM_CLOSE:
        if (ghRC)
            wglDeleteContext(ghRC);
        if (ghDC)
            ReleaseDC(hWnd, ghDC);
        ghRC = 0;
        ghDC = 0;

        DestroyWindow(hWnd);
        break;

    case WM_DESTROY:
        if (ghRC)
            wglDeleteContext(ghRC);
        if (ghDC)
            ReleaseDC(hWnd, ghDC);

        PostQuitMessage(0);
        break;

    case WM_KEYDOWN:
        switch (wParam) {
        case VK_LEFT:
            longinc += 0.F;
            break;
        case VK_RIGHT:
            longinc -= 0.F;
            break;
        case VK_UP:
            latinc += 0.F;
            break;
        case VK_DOWN:
            latinc -= 0.F;
            break;
        }

    case WM_RBUTTONDOWN:
        xPos = GET_X_LPARAM(lParam);
        yPos = GET_Y_LPARAM(lParam);
        SetCapture(hWnd);

        return 0;

    case WM_RBUTTONUP:
        ReleaseCapture();
        return 0;

    case WM_LBUTTONDOWN:
        xPos = GET_X_LPARAM(lParam);
        yPos = GET_Y_LPARAM(lParam);
        SetCapture(hWnd);

        return 0;

    case WM_LBUTTONUP:
        ReleaseCapture();
        return 0;

    case WM_MOUSEMOVE:
        xPos = GET_X_LPARAM(lParam);
        yPos = GET_Y_LPARAM(lParam);
        fwKeys = GET_KEYSTATE_WPARAM(wParam);
        if (fwKeys & MK_LBUTTON)
        {
        }
        return 0;
        
    case WM_MOUSEWHEEL:
        fwKeys = GET_KEYSTATE_WPARAM(wParam);
        zDelta = GET_WHEEL_DELTA_WPARAM(wParam);
        xPos = GET_X_LPARAM(lParam);
        yPos = GET_Y_LPARAM(lParam);
        mult = zDelta / 120;
        scaleX += mult*0.1f;
        scaleY += mult *0.1f;
        scaleZ += mult*0.1f;

    default:
        lRet = DefWindowProc(hWnd, uMsg, wParam, lParam);
        break;
    }

    return lRet;
}

BOOL bSetupPixelFormat(HDC hdc)
{
    PIXELFORMATDESCRIPTOR pfd, *ppfd;
    int pixelformat;

    ppfd = &pfd;

    ppfd->nSize = sizeof(PIXELFORMATDESCRIPTOR);
    ppfd->nVersion = 1;
    ppfd->dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL |
        PFD_DOUBLEBUFFER;
    ppfd->dwLayerMask = PFD_MAIN_PLANE;
    ppfd->iPixelType = PFD_TYPE_COLORINDEX;
    ppfd->cColorBits = 8;
    ppfd->cDepthBits = 16;
    ppfd->cAccumBits = 0;
    ppfd->cStencilBits = 0;

    pixelformat = ChoosePixelFormat(hdc, ppfd);

    if ((pixelformat = ChoosePixelFormat(hdc, ppfd)) == 0)
    {
        MessageBox(NULL, "ChoosePixelFormat failed", "Error", MB_OK);
        return FALSE;
    }

    if (SetPixelFormat(hdc, pixelformat, ppfd) == FALSE)
    {
        MessageBox(NULL, "SetPixelFormat failed", "Error", MB_OK);
        return FALSE;
    }

    return TRUE;
}

/* OpenGL code */

GLvoid resize(GLsizei width, GLsizei height)
{
    GLfloat aspect;

    glViewport(0, 0, width, height);

    aspect = (GLfloat)width / height;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, aspect, 3.0, 7.0);
    glMatrixMode(GL_MODELVIEW);
}

GLvoid createObjects()
{
    GLUquadricObj *quadObj;

    glNewList(GLOBE, GL_COMPILE);
    quadObj = gluNewQuadric();
    gluQuadricDrawStyle(quadObj, GLU_LINE);
    gluSphere(quadObj, 1.5, 16, 16);
    glEndList();

    glNewList(CONE, GL_COMPILE);
    quadObj = gluNewQuadric();
    gluQuadricDrawStyle(quadObj, GLU_FILL);
    gluQuadricNormals(quadObj, GLU_SMOOTH);
    gluCylinder(quadObj, 0.3, 0.0, 0.6, 15, 10);
    glEndList();

    glNewList(CYLINDER, GL_COMPILE);
    glPushMatrix();
    glRotatef((GLfloat)90.0, (GLfloat)1.0, (GLfloat)0.0, (GLfloat)0.0);
    glTranslatef((GLfloat)0.0, (GLfloat)0.0, (GLfloat)-1.0);
    quadObj = gluNewQuadric();
    gluQuadricDrawStyle(quadObj, GLU_FILL);
    gluQuadricNormals(quadObj, GLU_SMOOTH);
    gluCylinder(quadObj, 0.3, 0.3, 0.6, 12, 2);
    glPopMatrix();
    glEndList();

    glNewList(POINT_CLOUD, GL_COMPILE);
    glBegin(GL_POINTS);

    // read points from xyz file
    std::string file_path = "C:\\Projects\\OWIArmCamera\\KinectImages\\more\\img0.xyz";
    std::ifstream infile;
    infile.open(file_path, std::ifstream::in);
    while (!infile.eof())
    {
        float x, y, z;
        float r, g, b; // 0-255 range
        infile >> x;
        infile >> y;
        infile >> z;
        infile >> r;
        infile >> g;
        infile >> b;
        glColor3ub(r, g, b);
        glVertex3d(x, y, z);
    }
    infile.close();

    glEnd();
    glEndList();
}

GLvoid initializeGL(GLsizei width, GLsizei height)
{
    GLfloat     maxObjectSize, aspect;
    GLdouble    near_plane, far_plane;

    glClearIndex((GLfloat)BLACK_INDEX);
    glClearDepth(1.0);

    glEnable(GL_DEPTH_TEST);

    // white background
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    glMatrixMode(GL_PROJECTION);
    aspect = (GLfloat)width / height;
    gluPerspective(45.0, aspect, 3.0, 7.0);
    glMatrixMode(GL_MODELVIEW);

    near_plane = 3.0;
    far_plane = 7.0;
    maxObjectSize = 3.0F;
    radius = near_plane + maxObjectSize / 2.0;

    latitude = 0.0F;
    longitude = 0.0F;
    latinc = 0.0F;
    longinc = 0.0F;

    scaleX = 1.0f;
    scaleY = 1.0f;
    scaleZ = 1.0f;

    createObjects();
}

void polarView(GLdouble radius, GLdouble twist, GLdouble latitude,
    GLdouble longitude)
{
    glTranslated(0.0, 0.0, -radius);
    glRotated(-twist, 0.0, 0.0, 1.0);
    glRotated(-latitude, 1.0, 0.0, 0.0);
    glRotated(longitude, 0.0, 0.0, 1.0);

}

GLvoid zoomView(GLvoid)
{
    glScalef(scaleX, scaleY, scaleZ); // scale the matrix
}

GLvoid drawScene(GLvoid)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glPushMatrix();

#if 1
    latitude += latinc;
    longitude += longinc;

    polarView(radius, 0, latitude, longitude);
    zoomView();

    glIndexi(BLUE_INDEX);
    glCallList(POINT_CLOUD);
#else
    latitude += latinc;
    longitude += longinc;

    polarView(radius, 0, latitude, longitude);

    glIndexi(RED_INDEX);
    glCallList(CONE);

    glIndexi(BLUE_INDEX);
    glCallList(GLOBE);

    glIndexi(GREEN_INDEX);
    glPushMatrix();
    glTranslatef(0.8F, -0.65F, 0.0F);
    glRotatef(30.0F, 1.0F, 0.5F, 1.0F);
    glCallList(CYLINDER);
    glPopMatrix();
#endif

    glPopMatrix();

    SWAPBUFFERS;
}