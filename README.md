# Open Test Drive Unlimited for AARCH64 (based on OpenTDU)
Original repo: https://github.com/opentestdriveunlimited/OpenTestDriveUnlimited

**WORK IN PROGRESS**

The goal of this project is to bring TDU to handheld consoles running rocknix as a port.

**This project is not a replacement for the game (you still need to own a legal copy); and does not provide any game assets!**

<img src="/screenshots/carshowcase_exige.jpg" alt="GMCarShowcase" width="256" height="144"> <img src="/screenshots/worldviewer_3.jpg" alt="GMSandbox" width="256" height="144"> <img src="/screenshots/freeride.jpg" alt="GMFreeride" width="256" height="144"> <img src="/screenshots/login.jpg" alt="GMLogin" width="256" height="144"> <img src="/screenshots/cinviewer.jpg" alt="GMCinViewer" width="256" height="144"> <img src="/screenshots/boattour.jpg" alt="GMBoatTour" width="256" height="144"> <img src="/screenshots/living.jpg" alt="GMLiving" width="256" height="144"> <img src="/screenshots/garage.jpg" alt="GMGarage" width="256" height="144"> <img src="/screenshots/cockpit.jpg" alt="GMFreeride (cockpit)" width="256" height="144"> 

Updates of original project (updates/screenshots) can be found in this [thread](https://github.com/opentestdriveunlimited/OpenTestDriveUnlimited/discussions/175).



## Installation
1. Go to ports folder on rocknix device (/storage/roms/ports)
2. Create two folders:
/storage/roms/ports/tdu
/storage/roms/ports/tdu/EURO
3. Copy from your original 1.66A game copy installed legally on pc. All content from Euro folder to /storage/roms/ports/tdu/EURO on rocknix
and evrything from main game folder (folder with TestDriveUnlimited.exe) to /storage/roms/ports/tdu
4. Copy evrything from release archive to /storage/roms/ports/
5. In rocknix menu (emulationstation) press start > game settings > update gamelists
6. Go to ports (if you dont have portmaster installed install it first), then open OpenTDU

## Command Line Arguments
| Command | Description |
| ----------- |  ----------- |
| -w | Enable windowed mode |
| -vbl (value) | Set target vsync blank |
| -fps | Enable FPS counter |
| -pos | Enable debug position text overlay |
| -orb | Override ORB address for online login |
| -nopub | Disable GSPub |
| -gm (id) | Launch game with a specific game mode (see eGameMode) |
| -worldviewer | Launch world viewer |
| -carshowcase | Launch car showcase (alias for '-gm 7') |
| -cinviewer | Launch cinematic viewer (alias for '-gm 20') |
| -freeride | Launch cinematic viewer (alias for '-gm 1') |
| -login | Launch login screen (alias for '-gm 24') |
| -vk_enable_validation_layer | Create Vulkan render device with validation layer enabled |
| -break_on_assert_failure | Trigger debug breakpoint on assert failure |
| -disable_assert | Disable assert log/debug break |
| -norunlock | Disable single game instance run lock |
| -profile (name) | Launch game with a specific profile (must match savegame name) |
| -car (id) | Override the local player car in world game modes (e.g. freeride) with a car database ID |
| -silent | Disable logging (stdout, outpudebug and file) | 
| -log (path) | Redirect game logs to a specific file (default: ./opentdu.log) |
| -nolog | Disable logging to file (note: does not disable stdout/outputdebug) |
| -logtransient | Write transient log to file |
| -verbose | Enable verbose (aka Debug) logging. Adds significant runtime overhead |
| -maxfps (fps) | Set framecap (0: unlimited) (default: 0) |

## Debug Shortcuts
- F1 - Engine debug menu (services + managers)
- F2 - Pause simulation
- F3 - Active game mode debug menu

## Project progress
Progress is tracked on a game mode basis:
| GameMode ID  | GameMode  | Status |
| -----------  | ----------- | ----------- |
| 1| FreeRide|**Done**|
| 2| Map|**Done**|
| 3| Home|**Done**|
| 4| Shop|**Done**|
| 5| Mission|**In Progress**|
| 6| GameModeEdit|**In Progress**|
| 7| CarShowCase|**Done**|
| 8| InstantChallenge|**In Progress**|
| 9| TestCar|**Done**|
| 10| Garage|**Done**|
| 11| Living|**Done**|
| 12| Sandbox|**Done**|
| 13| Goodies|**Done**|
| 14| AvatarShowcase|**Done**|
| 15| DriveIn|**In Progress**|
| 16| Dressing|**Done**|
| 17| Aftermarket|**Done**|
| 18| VideoBumper|**Done**|
| 19| AITrainer|**In Progress**|
| 20| CinViewer|**Done**|
| 21| Estates|**Done**|
| 22| Club|**In Progress**|
| 23| Lobby|**In Progress**|
| 24| Login|**Done**|
| 25| OnlineReplay|**In Progress**|
| 26| Intro|**In Progress**|
| 27| OutroDemo|**Done**|
| 28| BoatTour|**Done**|
| 29| Jail|**Done**|
| 30| IntroDemo|**Done**|
