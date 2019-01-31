#ifndef Prefix_h
#define Prefix_h

#import <CocoaLumberjack/CocoaLumberjack.h>

#if DEBUG
static const DDLogLevel ddLogLevel = DDLogLevelVerbose;
#else
static const DDLogLevel ddLogLevel = DDLogLevelWarning;
#endif

#define REQUESTED_URL_KEY @"requestedURL"

#define PREFER_FPS 10
#define CVIMAGE_FPS 24

#endif /* Prefix_h */

