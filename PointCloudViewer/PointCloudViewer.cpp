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
#include <sstream>

#include <limits>
#include <cstddef>

#include <Eigen/Dense>

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
float translate_camera[3];
float translate_camera_speed[3];

float rotate_camera_angle;
float rotate_camera_direction[3];
float rotate_camera_speed;

float edge_length;

int last_mouse_pos_x;
int last_mouse_pos_y;

#define POINT_CLOUD 1

GLvoid resize(GLsizei, GLsizei);
GLvoid initializeGL(GLsizei, GLsizei);
GLvoid drawScene(GLvoid);

using namespace std;
using namespace Eigen;

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

void reset_settings() {
    translate_camera[0] = 0.0f;
    translate_camera[1] = 0.0f;
    translate_camera[2] = 0.0f;

    translate_camera_speed[0] = 0.01f;
    translate_camera_speed[1] = 0.01f;
    translate_camera_speed[2] = 0.01f;

    rotate_camera_angle = 0.0f;

    rotate_camera_direction[1] = 0.0f;
    rotate_camera_direction[1] = 0.0f;
    rotate_camera_direction[2] = 0.0f;

    rotate_camera_speed = 0.01f;

    edge_length = 0.1f;
}


int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
    MSG        msg;
    WNDCLASS   wndclass;

    LPWSTR *szArglist;
    int nArgs;
    int i;

    vector< string > arg_list;

    reset_settings();

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
        switch (wParam)
        {
        case VK_LEFT:
            // Process the LEFT ARROW key. 
            break;
        case VK_RIGHT:
            // Process the RIGHT ARROW key. 
            break;
        case VK_UP:
            // Process the UP ARROW key. 
            break;
        case VK_DOWN:
            // Process the DOWN ARROW key. 
            break;
        case VK_HOME:
            // Process the HOME key. 
            break;
        case VK_END:
            // Process the END key. 
            break;
        case VK_INSERT:
            // Process the INS key. 
            break;
        case VK_DELETE:
            // Process the DEL key. 
            break;
        case VK_F2:
            // Process the F2 key. 
            break;
        default:
            // Process other non-character keystrokes. 
            break;
        }

    case WM_CHAR:
        switch (wParam)
        {
        case 0x08:
            // Process a backspace. 
            break;
        case 0x0A:
            // Process a linefeed. 
            break;
        case 0x1B:
            // Process an escape. 
            PostMessage(hWnd, WM_CLOSE, 0, 0);
            break;
        case 0x09:
            // Process a tab. 
            break;
        case 0x0D:
            // Process a carriage return. 
            break;
        case 0x52:
        case 0x72:
            // Process R, r
            reset_settings();
        default:
            // Process displayable characters. 
//            ch = (TCHAR)wParam;
            break;
        }

    case WM_RBUTTONDOWN:
        xPos = GET_X_LPARAM(lParam);
        yPos = GET_Y_LPARAM(lParam);

        last_mouse_pos_x = xPos;
        last_mouse_pos_y = yPos;

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
        if (fwKeys & MK_LBUTTON || fwKeys & MK_RBUTTON)
        {
            if (fwKeys & MK_LBUTTON) {
                translate_camera[0] -= (last_mouse_pos_x - xPos)*translate_camera_speed[0];
                translate_camera[1] += (last_mouse_pos_y - yPos)*translate_camera_speed[1];
            }
            else {
                rotate_camera_direction[0] += (last_mouse_pos_y - yPos)*rotate_camera_speed;
                rotate_camera_direction[1] -= (last_mouse_pos_x - xPos)*rotate_camera_speed;
                rotate_camera_angle = sqrt(rotate_camera_direction[0] * rotate_camera_direction[0] +
                    rotate_camera_direction[1] * rotate_camera_direction[1] +
                    rotate_camera_direction[2] * rotate_camera_direction[2]);
            }

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
        translate_camera[2] += mult*translate_camera_speed[2];

    default:
        lRet = (LONG)DefWindowProc(hWnd, uMsg, wParam, lParam);
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

    // the OpenGL projection transformation (not to be confused with the projective transformation in the computer vision) transforms 
    // all vertex data from the eye coordinates to the clip coordinates.

    float zNear = (max_obj_coord[2] - min_obj_coord[2]) / 100.0f;;
    float zFar = zNear + (max_obj_coord[2] - min_obj_coord[2])*3.0f;
    gluPerspective(45.0, aspect, zNear, zFar);
/*
    glFrustum(min_obj_coord[0], max_obj_coord[0],
        min_obj_coord[1], max_obj_coord[1],
        (max_obj_coord[2] - min_obj_coord[2])/2.0, (max_obj_coord[2] - min_obj_coord[2])*3.0 / 2.0);
        */
    glMatrixMode(GL_MODELVIEW);
}

GLvoid createObjects()
{
    glNewList(POINT_CLOUD, GL_COMPILE);

    for (int i = 0; i < 3; i++) {
        min_obj_coord[i] = FLT_MAX;
        max_obj_coord[i] = FLT_MIN;
    }

    // read points from xyz file
    string line;
    std::ifstream infile;
    infile.open(point_cloud_file_path, std::ifstream::in);
    while (std::getline(infile, line)) {
       
        std::stringstream linestream(line);
        int count = 0;
        float obj_coord[3];
        float obj_color[3]; // 0-255 range
        float edge_direction[3];
        for (int i = 0; i < 3; i++) {
            if (linestream >> obj_coord[i])
                count++;
        }
        for (int i = 0; i < 3; i++) {
            if (linestream >> obj_color[i])
                count++;
        }

        for (int i = 0; i < 3; i++) {
            if (linestream >> edge_direction[i]) {
                count++;
            }
        }

        glBegin(GL_POINTS);
        glColor3ub((GLubyte)obj_color[0], (GLubyte)obj_color[1], (GLubyte)obj_color[2]);
        glVertex3f(obj_coord[0], obj_coord[1], obj_coord[2]);
        glEnd();

        if (count >= 9) {
            glBegin(GL_LINES);
            glColor3ub(255, 0, 0);
            Vector3f v1(obj_coord[0], obj_coord[1], obj_coord[2]);
            Vector3f vEdge(edge_direction[0], edge_direction[1], edge_direction[2]);
            vEdge.normalize();
            Vector3f v2 = v1 + vEdge * edge_length;

            glVertex3f(v1(0), v1(1), v1(2));
            glVertex3f(v2(0), v2(1), v2(2));

            glEnd();
        }

        for (int i = 0; i < 3; i++) {
            if (obj_coord[i] < min_obj_coord[i]) min_obj_coord[i] = obj_coord[i];
            if (obj_coord[i] > max_obj_coord[i]) max_obj_coord[i] = obj_coord[i];
        }
    }
    infile.close();

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
    // the camera originally (in initializeGL) was shifted in the positive z direction (of the point cloud reference frame) 
    // from the point cloud center and looking at the point cloud center. 
    // The camera is always at the origin of the camera reference frame looking in its negative z direction, where the point cloud is supposed to be.
    // when the positive Z translation is applied to the point cloud points in the camera reference frame, 
    // their negative z coordinates get smaller in magnitude, they get closer to the origin, which creates zooming effect
    glTranslatef(translate_camera[0], translate_camera[1], translate_camera[2]);
    // rotate after translation
    glRotatef((GLfloat)rotate_camera_angle, rotate_camera_direction[0], rotate_camera_direction[1], rotate_camera_direction[2]);

    glCallList(POINT_CLOUD);

    glPopMatrix();

    SWAPBUFFERS;
}