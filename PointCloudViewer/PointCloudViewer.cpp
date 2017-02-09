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

#include "point_cloud.h"

#define MAIN_LOGGER
#define LOGGING_LEVEL_1
#include "logger.hpp"

using namespace std;
using namespace Eigen;
using namespace pcv;

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
void refresh_display_lists();

/* OpenGL globals, defines, and prototypes */
float translate_camera[3];

Eigen::Vector3f translate_camera_speed;

float rotate_camera_angle;
float rotate_camera_direction[3];
float rotate_camera_speed;

float edge_length;

float gl_point_size;

int last_mouse_pos_x;
int last_mouse_pos_y;

struct c_viewport {
    c_viewport() {
    }
    c_viewport(int x, int y, int width, int height) {
        this->x = x;
        this->y = y;
        this->width = width;
        this->height = height;
    }
    int x;
    int y;
    int width;
    int height;
};
c_viewport viewport;

#define POINT_CLOUD 1

GLvoid resize_viewport(GLsizei, GLsizei);
GLvoid initializeGL(GLsizei, GLsizei);
GLvoid drawScene(GLvoid);

string input_point_cloud_file_path;
string output_point_cloud_file_path;

c_point_cloud point_cloud;

enum class c_interactive_mode: int
{
    idle,
    selecting_points_to_hide,
    updating_points_to_hide,
    selecting_points_to_keep,
    updating_points_to_keep,
    filter_points_by_color
};
c_interactive_mode interactive_mode = c_interactive_mode::idle;

Vector2i min_visible;
Vector2i max_visible;

bool get_command_line_options(vector< string >& arg_list) {
    int i;
    // requests parameters to be passed in pairs
    if (arg_list.size() % 2)
        return false;

    for (i = 0; i < arg_list.size(); i = i + 2) {
        if (arg_list[i] == "-i") {
            input_point_cloud_file_path = arg_list[i + 1];
        }
        if (arg_list[i] == "-o") {
            output_point_cloud_file_path = arg_list[i + 1];
        }
    }

    if (output_point_cloud_file_path.size() == 0) {
        output_point_cloud_file_path = input_point_cloud_file_path;
    }

    return true;
}

void reset_settings() {
    translate_camera[0] = 0.0f;
    translate_camera[1] = 0.0f;
    translate_camera[2] = 0.0f;

    translate_camera_speed = Vector3f(0.02f, 0.02f, 0.05f);
    
    rotate_camera_angle = 0.0f;

    rotate_camera_direction[1] = 0.0f;
    rotate_camera_direction[1] = 0.0f;
    rotate_camera_direction[2] = 0.0f;

    rotate_camera_speed = 0.05f;

    edge_length = 0.01f;

    gl_point_size = 3.0f;

    point_cloud.reset_visibility();

    interactive_mode = c_interactive_mode::idle;
}

bool get_point_screen_coordinate(const c_point_cloud_point& point, const Matrix4f& model_view_matrix, const Matrix4f& projection_matrix, Vector2i& point_screen_coordinates) {
    Vector4f point_coordinates;
    point_coordinates(3) = 1.0f;
    for (int i = 0; i < 3; i++) {
        point_coordinates(i) = point.X(i);
    }

    // see http://www.songho.ca/opengl/gl_transform.html#projection and http://webglfactory.blogspot.com/2011/05/how-to-convert-world-to-screen.html
    Vector4f clip_coordinates = projection_matrix*model_view_matrix*point_coordinates;
    Vector3f normalized_device_coordinates(clip_coordinates(0) / clip_coordinates(3), clip_coordinates(1) / clip_coordinates(3), clip_coordinates(2) / clip_coordinates(3));
    point_screen_coordinates(0) = (int)((float)viewport.width*(normalized_device_coordinates(0) + 1.0) / 2.0) + viewport.x;
    point_screen_coordinates(1) = (int)((float)viewport.height*(-normalized_device_coordinates(1) + 1.0) / 2.0) + viewport.y;
    return true;
}

void update_points_visibility() {
    Vector2i point_screen_coordinates;

    GLfloat model_view_matrix_raw[16];
    glGetFloatv(GL_MODELVIEW_MATRIX, (GLfloat*)&model_view_matrix_raw);
    Matrix4f model_view_matrix;
    for (int i = 0; i < 4; i++) {
        model_view_matrix.col(i) << model_view_matrix_raw[i * 4], model_view_matrix_raw[i * 4 + 1], model_view_matrix_raw[i * 4 + 2], model_view_matrix_raw[i * 4 + 3];
    }

    GLfloat projection_matrix_raw[16];
    glGetFloatv(GL_PROJECTION_MATRIX, (GLfloat*)&projection_matrix_raw);
    Matrix4f projection_matrix;
    for (int i = 0; i < 4; i++) {
        projection_matrix.col(i) << projection_matrix_raw[i * 4], projection_matrix_raw[i * 4 + 1], projection_matrix_raw[i * 4 + 2], projection_matrix_raw[i * 4 + 3];
    }

    for (auto it = point_cloud.points.begin(); it != point_cloud.points.end(); ++it) {
        if (!it->visible)
            continue;

        if (interactive_mode == c_interactive_mode::filter_points_by_color) {
            if (it->Clr != Vector3f(255, 0, 0)) {
                it->visible = 0;
            }
            continue;
        }

        if (get_point_screen_coordinate(*it, model_view_matrix, projection_matrix, point_screen_coordinates)) {
            bool point_in_selected_box = true;
            for (int i = 0; i < 2; i++) {
                if (point_screen_coordinates(i) < min_visible(i) || point_screen_coordinates(i) > max_visible(i)) {
                    point_in_selected_box = false;
                    break;
                }
            }

            if (interactive_mode == c_interactive_mode::updating_points_to_keep) {
                if (!point_in_selected_box) {
                    it->visible = 0;
                }
            } 
            else
            if (interactive_mode == c_interactive_mode::updating_points_to_hide) {
                if (point_in_selected_box) {
                    it->visible = 0;
                }
            }
        }
    }

    refresh_display_lists();
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
    LONG    lRet = 0;
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
        return 0;

    case WM_SIZE:
        GetClientRect(hWnd, &rect);
        resize_viewport(rect.right, rect.bottom);
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
        break;
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
        case 0x45:
        case 0x65:
            // Process E, e
            break;
        case 0x46:
        case 0x66:
            // Process F, f
            interactive_mode = c_interactive_mode::filter_points_by_color;
            break;
        case 0x48:
        case 0x68:
            // Process H, h
            interactive_mode = c_interactive_mode::selecting_points_to_hide;
            min_visible = Vector2i(INT_MAX, INT_MAX);
            max_visible = Vector2i(INT_MIN, INT_MIN);
            break;
        case 0x4B:
        case 0x6B:
            // Process K, k
            interactive_mode = c_interactive_mode::selecting_points_to_keep;
            min_visible = Vector2i(INT_MAX, INT_MAX);
            max_visible = Vector2i(INT_MIN, INT_MIN);
            break;
        case 0x52:
        case 0x72:
            // Process R, r
            reset_settings();
            refresh_display_lists();
            break;
        case 0x53:
        case 0x73:
            // Process S, s
            if (output_point_cloud_file_path.size() > 0) {
                point_cloud.write_point_cloud_file(output_point_cloud_file_path);
            }
            break;

        default:
            // Process displayable characters. 
//            ch = (TCHAR)wParam;
            break;
        }
        break;

    case WM_RBUTTONDOWN:
        xPos = GET_X_LPARAM(lParam);
        yPos = GET_Y_LPARAM(lParam);

        last_mouse_pos_x = xPos;
        last_mouse_pos_y = yPos;

        SetCapture(hWnd);

        return 0;

    case WM_RBUTTONUP:
        ReleaseCapture();
        break;

    case WM_LBUTTONDOWN:
        xPos = GET_X_LPARAM(lParam);
        yPos = GET_Y_LPARAM(lParam);

        last_mouse_pos_x = xPos;
        last_mouse_pos_y = yPos;

        SetCapture(hWnd);

        break;

    case WM_LBUTTONUP:
        if (interactive_mode == c_interactive_mode::selecting_points_to_hide) {
            interactive_mode = c_interactive_mode::updating_points_to_hide;
        }
        if (interactive_mode == c_interactive_mode::selecting_points_to_keep) {
            interactive_mode = c_interactive_mode::updating_points_to_keep;
        }
        
        ReleaseCapture();
        break;

    case WM_MOUSEMOVE:
        xPos = GET_X_LPARAM(lParam);
        yPos = GET_Y_LPARAM(lParam);
        fwKeys = GET_KEYSTATE_WPARAM(wParam);
        if (interactive_mode == c_interactive_mode::selecting_points_to_hide ||
            interactive_mode == c_interactive_mode::selecting_points_to_keep
            ) {
            if (fwKeys & MK_LBUTTON) {
                if (xPos < min_visible(0))
                    min_visible(0) = xPos;
                if (yPos < min_visible(1))
                    min_visible(1) = yPos;
                if (xPos > max_visible(0))
                    max_visible(0) = xPos;
                if (yPos > max_visible(1))
                    max_visible(1) = yPos;
            }
        } 
        else 
        if (interactive_mode == c_interactive_mode::idle) {
            if (fwKeys & MK_LBUTTON) {
                translate_camera[0] -= (last_mouse_pos_x - xPos)*translate_camera_speed(0);
                translate_camera[1] += (last_mouse_pos_y - yPos)*translate_camera_speed(1);
            }
            else 
            if (fwKeys & MK_RBUTTON) {
                rotate_camera_direction[0] += (last_mouse_pos_y - yPos)*rotate_camera_speed;
                rotate_camera_direction[1] -= (last_mouse_pos_x - xPos)*rotate_camera_speed;
                rotate_camera_angle = sqrt(rotate_camera_direction[0] * rotate_camera_direction[0] +
                    rotate_camera_direction[1] * rotate_camera_direction[1] +
                    rotate_camera_direction[2] * rotate_camera_direction[2]);
            }
        }

        last_mouse_pos_x = xPos;
        last_mouse_pos_y = yPos;
        break;
        
    case WM_MOUSEWHEEL:
        fwKeys = GET_KEYSTATE_WPARAM(wParam);
        zDelta = GET_WHEEL_DELTA_WPARAM(wParam);
        xPos = GET_X_LPARAM(lParam);
        yPos = GET_Y_LPARAM(lParam);
        mult = zDelta / 120;
        if (interactive_mode == c_interactive_mode::idle) {
            translate_camera[2] += mult*translate_camera_speed(2);
        }
        break;

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
GLvoid resize_viewport(GLsizei width, GLsizei height)
{
    GLfloat aspect;

    viewport = c_viewport(0, 0, width, height);
    glViewport(viewport.x, viewport.y, viewport.width, viewport.height);

    aspect = (GLfloat)width / height;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // the OpenGL projection transformation (not to be confused with the projective transformation in the computer vision) transforms 
    // all vertex data from the eye coordinates to the clip coordinates.

    float zNear = (point_cloud.max_coord(2) - point_cloud.min_coord(2)) / 100.0f;;
    float zFar = FLT_MAX;  // zNear + (point_cloud.max_coord(2) - point_cloud.min_coord(2))*10.0f;
    gluPerspective(45.0, aspect, zNear, zFar);

    glMatrixMode(GL_MODELVIEW);
}

void refresh_display_lists() {
    glNewList(POINT_CLOUD, GL_COMPILE);

    for (auto it = point_cloud.points.begin(); it != point_cloud.points.end(); ++it) {
        if (it->visible == 0)
            continue;
        glBegin(GL_POINTS);
        glColor3ub((GLubyte)it->Clr(0), (GLubyte)it->Clr(1), (GLubyte)it->Clr(2));
        glVertex3f(it->X(0), it->X(1), it->X(2));
        glEnd();

        if (it->Vector != Vector3f::Zero()) {
            glBegin(GL_LINES);
            glColor3ub(255, 0, 0);
            Vector3f normalized_edge = it->Vector;
            normalized_edge.normalize();
            Vector3f v = it->X + normalized_edge * edge_length;

            glVertex3f(it->X(0), it->X(1), it->X(2));
            glVertex3f(v(0), v(1), v(2));

            glEnd();
        }
    }

    glEndList();
}

GLvoid initializeGL(GLsizei width, GLsizei height)
{
    point_cloud.read_point_cloud_file(input_point_cloud_file_path);

    glClearIndex((GLfloat)BLACK_INDEX);
    glClearDepth(1.0);

    glEnable(GL_DEPTH_TEST);

    // white background
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    float center_obj_coord[3];
    for (int i = 0; i < 3; i++) {
        center_obj_coord[i] = (point_cloud.min_coord(i) + point_cloud.max_coord(i)) / 2.0f;
    }

    // position the camera looking at the object center
    // note, that in the opengl the camera looks at the negative z direction of the camera reference frame
    float camera_shift = (point_cloud.max_coord(2) - point_cloud.min_coord(2));
    gluLookAt(center_obj_coord[0], center_obj_coord[1], center_obj_coord[2] + camera_shift,
        center_obj_coord[0], center_obj_coord[1], center_obj_coord[2],
        0.0, 1.0, 0.0);

#ifdef LOGGING
//    LOG("initializeGL: point_cloud.points.size = ", point_cloud.points.size(), " width = ", width, " height = ", height);
#endif
    resize_viewport(width, height);

    refresh_display_lists();

    glPointSize(gl_point_size);
}

GLvoid drawScene(GLvoid)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

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

    if (interactive_mode == c_interactive_mode::updating_points_to_hide ||
        interactive_mode == c_interactive_mode::updating_points_to_keep ||
        interactive_mode == c_interactive_mode::filter_points_by_color
        ) {
        update_points_visibility();
        interactive_mode = c_interactive_mode::idle;
    }

    glCallList(POINT_CLOUD);

    glPopMatrix();

    SWAPBUFFERS;
}