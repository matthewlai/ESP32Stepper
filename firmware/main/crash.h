#ifndef CRASH_H
#define CRASH_H

void CrashHandler(const String& msg);

#define CRASH_AND_BURN(msg) CrashHandler(String(__FILE__) + "(" + String(__LINE__) + "): " + (msg));

#endif  // CRASH_H
