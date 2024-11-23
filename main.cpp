#define RAYGUI_IMPLEMENTATION

#include <iostream>
#include "raylib.h"
#include "simulation/planet.h"
#include "simulation/simulation_governor.h"
#include "raygui.h"
int main()
{
    const int screenWidth = 800;
    const int screenHeight = 450;
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);    // Window configuration flags
    InitWindow(screenWidth, screenHeight, "simulation test");
    Camera camera = { { 5.0f, 5.0f, 5.0f }, { 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, 45.0f, 0 };
    SetTargetFPS(60);
    std::vector<std::shared_ptr<sim::RigidBody>> bodies = {};
    std::mutex mtx;
    sim::SimulationGovernor governor = sim::SimulationGovernor(2,1000000);

        std::cout<<"Simulation not loaded"<<std::endl;
        /*Planet planet = new Planet(Vector3{3,0,0},10000,Vector3{0.00001,0,0},1,BLACK,"Earth",1);
        Planet planet2 = Planet(Vector3{-3,0,0},10,Vector3{-0.00001,0,0},1,RED,"Moon",2);*/

        bodies.push_back(std::make_shared<Planet>(Vector3{-3, 0, 0}, 1000, Vector3{-0.00004, 0, 0}, 1,RED, "Moon", 2));
        bodies.push_back(std::make_shared<Planet>(Vector3{3, 0, 0}, 10000, Vector3{0.00016, 0, 0}, 1,BLACK, "Earth", 1));
    bodies.push_back(std::make_shared<Planet>(Vector3{3, 0, 9}, 1000000, Vector3{-0.00001, 0, 0}, 1,YELLOW, "Sun", 2));
    bodies.push_back(std::make_shared<Planet>(Vector3{3, 2, 0}, 10000, Vector3{0.00002, 0.000005, 0}, 1,GREEN, "Yoo", 1));
    bodies.push_back(std::make_shared<Planet>(Vector3{3, -6, 0}, 1000, Vector3{-0.00001, 0, 0}, 1,VIOLET, "Eve", 2));
    bodies.push_back(std::make_shared<Planet>(Vector3{3, 0, 5}, 10000, Vector3{0.00032, 0, 0.00032}, 1,ORANGE, "Mars", 1));
    bodies.push_back(std::make_shared<Planet>(Vector3{-3, -3, 0}, 1000, Vector3{-0.00001, 0, 0}, 1,BLUE, "Neptune", 2));
    bodies.push_back(std::make_shared<Planet>(Vector3{1, -1, 5}, 10000, Vector3{0.00002, 0, 0}, 1,GRAY, "Pluto", 1));
        governor.setBodies(bodies);
        governor.loadSimulation();
    try {
        governor.startSimulation(mtx);
    }
    catch (std::exception e) {
        std::cout<<e.what()<<std::endl;
    }
    //sim::RigidBody* yes = &planet;
    //Planet* z= dynamic_cast<Planet*>(yes);
    std::cout<<"WEEEEEEEe"<<std::endl;

    //--------------------------------------------------------------------------------------
    std::cout<<GetMonitorRefreshRate(0)<<std::endl;
    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {

        std::cout<<"WEEEEEEEe"<<std::endl;
        //HideCursor();
        ClearBackground(RAYWHITE);
        UpdateCamera(&camera,CAMERA_FIRST_PERSON);
        std::cout<<"WEEEEEEEe"<<std::endl;
        // Update
        //----------------------------------------------------------------------------------
        // TODO: Update your variables here
        //----------------------------------------------------------------------------------
        Mesh mesh = GenMeshPlane(2, 2, 1, 1);
        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

        /*auto bodiz= governor.getBodies();
        //auto a= dynamic_cast<Planet*>(bodiz[0]);*/
        std::cout<<"WEEEEEEEe"<<std::endl;

        std::cout<<"WEEEEEEEe"<<std::endl;

        BeginMode3D(camera);
        Vector3 position = { 0.0f, 0.0f, 0.0f };
        //DrawModel(LoadModelFromMesh(GenMeshCube(5.0f,5.0f,5.0f)), position, 1.0f, WHITE);

        {
        std::lock_guard<std::mutex> lock(mtx);
        for (auto rigid_body: governor.bodies_) {
            try {
                Planet* we =dynamic_cast<Planet*>(rigid_body.get());
                DrawModel(LoadModelFromMesh(GenMeshSphere(1.0f,10.0f,10.0f)), we->position, 1.0f, we->color);
                std::cout<<"model drawn"<<we->name<<std::endl;
            }catch (error_t) {
                std::cout<<"Error"<<std::endl;
            }
        }
        }
        DrawGrid(500, 1.0);

        EndMode3D();

        Color color = {255, 0, 0, 255};
        //GuiLoadStyle("styles/light.rgs");
        //GuiLoadStyleDark();


        GuiSetStyle(TEXTBOX, TEXT_ALIGNMENT, TEXT_ALIGN_LEFT);
        GuiPanel((Rectangle){ 10, 10, 200, static_cast<float>(GetRenderHeight())-100 }, "Panel 1");
        GuiCheckBox((Rectangle){ 25, 60, 15, 15 }, "Is running", &governor.is_running);

        GuiGroupBox((Rectangle){ 25,90 , 125, 130 }, "Simulation controls");
        //GuiLock();
        GuiSetState(STATE_NORMAL); if (GuiButton((Rectangle){ 30, 100, 115, 30 }, "Pause simulation")) {governor.pauseSimulation();}
        GuiSetState(STATE_NORMAL); if (GuiButton((Rectangle){ 30, 140, 115, 30 }, "Start simulation")) {governor.startSimulation(mtx);}
        GuiSetState(STATE_NORMAL); if (GuiButton((Rectangle){ 30, 180, 115, 30 }, "Save simulation")) {governor.saveSimulation(mtx);}
        //GuiUnlock();
        //DrawText("Congrats! You created your first window!", 190, 200, 20, LIGHTGRAY);
        EndDrawing();

        //----------------------------------------------------------------------------------
    }
    governor.pauseSimulation();
    //governor.saveSimulation();
    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}
