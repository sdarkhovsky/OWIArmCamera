// PointCloudViewer.cpp : Defines the entry point for the application.
//

/*
* Example of a Windows OpenGL program.
* The OpenGL code is the same as that used in
* the X Window System sample
*/
#include <windows.h> 
#include <windowsx.h>
#include <shellapi.h>

#include <stdlib.h>

#include <GL/gl.h> 
#include <GL/glu.h> 

#include <vector>
#include <iostream>
#include <fstream>
#include <string>

#include <limits>
#include <cstddef>

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
GLfloat translate[3] = { 0, 0, 0 };
float translate_speed[3] = { 0.01f, 0.01f, 0.1f };

int last_mouse_pos_x;
int last_mouse_pos_y;

#define POINT_CLOUD 1

GLvoid resize(GLsizei, GLsizei);
GLvoid initializeGL(GLsizei, GLsizei);
GLvoid drawScene(GLvoid);

using namespace std;

string point_cloud_file_path;

float min_obj_coord[3];
float max_obj_coord[3];

bool get_command_line_options(vector< string >& arg_list) {
    int i;
    // requests parameters to be passed in pairs
    if (arg_list.size() % 2)
        return false;

    for (i = 0; i < arg_list.size(); i = i + 2) {
        if (arg_list[i] == "-p") {
            point_cloud_file_path = arg_list[i + 1];
        }
    }

    return true;
}


int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
    MSG        msg;
    WNDCLASS   wndclass;

    LPWSTR *szArglist;
    int nArgs;
    int i;

    vector< string > arg_list;

    szArglist = CommandLineToArgvW(GetCommandLineW(), &nArgs);
    if (NULL == szArglist) return 0;
    // skip the first argument, which is the full path of the executable
    for (i = 1; i < nArgs; i++) {
        size_t buf_len = wcslen(szArglist[i])+1;  // with the null terminator
        char* buf = (char*)calloc(buf_len+1, 1);
        if (!buf) return 0;
        size_t ret_len;
        errno_t ret = wcstombs_s(&ret_len, buf, buf_len, szArglist[i], buf_len);
        string s = buf;
        arg_list.push_back(s);
        free(buf);
    }
    // Free memory allocated for CommandLineToArgvW arguments.
    LocalFree(szArglist);

    if (!get_command_line_options(arg_list))
        return 0;

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
//    ShowWindow(ghWnd, nCmdShow);

    ShowWindow(ghWnd, SW_MAXIMIZE);
    
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
            break;
        case VK_RIGHT:
            break;
        case VK_UP:
            break;
        case VK_DOWN:
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

        last_mouse_pos_x = xPos;
        last_mouse_pos_y = yPos;

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
            translate[0] -= (last_mouse_pos_x - xPos)*translate_speed[0];
            translate[1] += (last_mouse_pos_y - yPos)*translate_speed[1];
            last_mouse_pos_x = xPos;
            last_mouse_pos_y = yPos;
        }
        return 0;
        
    case WM_MOUSEWHEEL:
        fwKeys = GET_KEYSTATE_WPARAM(wParam);
        zDelta = GET_WHEEL_DELTA_WPARAM(wParam);
        xPos = GET_X_LPARAM(lParam);
        yPos = GET_Y_LPARAM(lParam);
        mult = zDelta / 120;
        translate[2] += mult*translate_speed[2];

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
    glFrustum(min_obj_coord[0], max_obj_coord[0],
        min_obj_coord[1], max_obj_coord[1],
        (max_obj_coord[2] - min_obj_coord[2])/2.0, (max_obj_coord[2] - min_obj_coord[2])*3.0 / 2.0);
    glMatrixMode(GL_MODELVIEW);
}

GLvoid createObjects()
{
    glNewList(POINT_CLOUD, GL_COMPILE);
    glBegin(GL_POINTS);

    // read points from xyz file
    for (int i = 0; i < 3; i++) {
        min_obj_coord[i] = FLT_MAX;
        max_obj_coord[i] = FLT_MIN;
    }

    std::ifstream infile;
    infile.open(point_cloud_file_path, std::ifstream::in);
    while (!infile.eof())
    {
        float obj_coord[3];
        float obj_color[3]; // 0-255 range
        for (int i = 0; i < 3; i++) {
            infile >> obj_coord[i];
        }
        for (int i = 0; i < 3; i++) {
            infile >> obj_color[i];
        }
        glColor3ub((GLubyte)obj_color[0], (GLubyte)obj_color[1], (GLubyte)obj_color[2]);
        glVertex3f(obj_coord[0], obj_coord[1], obj_coord[2]);

        for (int i = 0; i < 3; i++) {
            if (obj_coord[i] < min_obj_coord[i]) min_obj_coord[i] = obj_coord[i];
            if (obj_coord[i] > max_obj_coord[i]) max_obj_coord[i] = obj_coord[i];
        }
    }
    infile.close();

    glEnd();
    glEndList();
}

GLvoid initializeGL(GLsizei width, GLsizei height)
{
    glClearIndex((GLfloat)BLACK_INDEX);
    glClearDepth(1.0);

    glEnable(GL_DEPTH_TEST);

    // white background
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    createObjects();

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    float center_obj_coord[3];
    for (int i = 0; i < 3; i++) {
        center_obj_coord[i] = (min_obj_coord[i] + max_obj_coord[i]) / 2.0f;
    }

    // position the camera looking at the object center
    // note, that in the opengl the camera looks at the negative z direction of the camera reference frame
    float camera_shift = (max_obj_coord[2] - min_obj_coord[2]);
    gluLookAt(center_obj_coord[0], center_obj_coord[1], center_obj_coord[2] + camera_shift,
        center_obj_coord[0], center_obj_coord[1], center_obj_coord[2],
        0.0, 1.0, 0.0);

    resize(width, height);
}

GLvoid drawScene(GLvoid)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//    GLfloat matrix[4][4];
//    glGetFloatv(GL_MODELVIEW_MATRIX, (GLfloat*)matrix);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    // zoom the view
    glTranslatef(translate[0], translate[1], translate[2]);

    glCallList(POINT_CLOUD);

    glPopMatrix();

    SWAPBUFFERS;
}