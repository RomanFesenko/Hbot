HBot - mode for Half-Life 1
Readme File
13/10/2020

******************************************************
I.   Mode description.

HBot  -  artificial player for multiplayer mode Half-Life 1. 
Bot can: move along the map, collected items and shoot to enemies, use HW- and AW walls, use elevators, move along vertical staircase, overcome the obstacle. Bot fight "honest", use solely information, available for human - player in equal situation.
Bot can't: use grenades and mines all types, interact with other bots and human - player.

II.  Compilation and setup.

   Content of  hbot_dll folder  compilate in  separate  hbot.dll,your need compiler with C++20 support (will need support for C ++ 20 concepts);  hl_dll folder content  - compilate with standart Single-Player Source  HL SDK compiler, with replace original sources with equal name (player.cpp, client.cpp, h_export.cpp ).
   For setup: hl.dll and hbot.dll copy to   .../Valve/valve/dlls, with replace original hl.dll; folder  BotFiles, which contain waypoints files ( for bot navigation) and bot settings (in bot_data.txt) -  copy to  .../Valve/valve.

III. Bot commands.

"add_bot" -  add new bot in game.
"set_visible_0"  human-player become  invisible  for all bots. 
"set_visible_1"  human-player become  visible  for all bots. 
"set_difficult_X"  where X=[0 , 5] - set  bots accuracy, greater X correspond to more accuracy bots shooting.
     For more detail bot settings, use bot_data.txt file in BotFiles,where possible to set bot name, accuracy and aggressive in range [0.0 , 1.0].
