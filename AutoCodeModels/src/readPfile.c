#include "readPfile.h"
#include "Parameters.h"
#include <stdio.h>
#if defined WIN32
#include <fcntl.h>
#include <io.h>
#endif

extern BUS_PARAM_SimPar SimPar;

// Read serialized Parameter File
int readPFile (const char *filename) {
  // read SimPar binary file from standard input
#if defined WIN32
  int result = _setmode( _fileno( stdin ), _O_BINARY );
  if( result == -1 )
    perror( "Cannot set mode" );
  else
    fprintf(stderr, "'stdin' successfully changed to binary mode\n" );  
#endif

  size_t nNumRead = fread(&SimPar, sizeof(BUS_PARAM_SimPar), 1, stdin);
  fprintf(stderr, "Num Read = %zu, size of param = %zu\n", nNumRead, sizeof(BUS_PARAM_SimPar));
  return(0);
}
