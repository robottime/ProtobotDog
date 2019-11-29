#ifndef _CLI_H_
#define _CLI_H_

#define MAX_MSG_SIZE 60
#include <stdint.h>
#include <Arduino.h>
#include <HardwareSerial.h>

#define MAX_MSG_SIZE 60
#define MAX_PATH_LEN 20

// command line structure
typedef struct _cmd_t
{
  char *cmd;
  void (*func)(int argc, char **argv);
  struct _cmd_t *next;
} cmd_t;

class Cli {
 public:
  Cli() {}
  void setup(Stream *serial, char path[]="cli") {
    _serial = serial;
    _msg_ptr = _msg;
    _cmd_tbl_list = NULL;
    strcpy(_path, path);
    prompt();
  }
  void read() {
    while(_serial->available()) {
      handler();
    }
  }
  void newCmd(const char *name, void (*func)(int argc, char **argv)) {
    // alloc memory for command struct
    _cmd_tbl = (cmd_t *)malloc(sizeof(cmd_t));

    // alloc memory for command name
    char *cmd_name = (char *)malloc(strlen(name) + 1);

    // copy command name
    strcpy(cmd_name, name);

    // terminate the command name
    cmd_name[strlen(name)] = '\0';

    // fill out structure
    _cmd_tbl->cmd = cmd_name;
    _cmd_tbl->func = func;
    _cmd_tbl->next = _cmd_tbl_list;
    _cmd_tbl_list = _cmd_tbl;
  }
  void setPath(char *path) {
    strcpy(_path, path);
  }
 private:
  Stream *_serial;
  char _msg[MAX_MSG_SIZE];
  char *_msg_ptr;
  cmd_t *_cmd_tbl_list;
  cmd_t *_cmd_tbl;

  char _path[MAX_PATH_LEN];
  const char _err_msg[24] = "Command not recognized."; 

  void prompt() {
    _serial->flush();
    _serial->print(_path);
    _serial->print(" >> ");
  }

  void parse(char *cmd) {
    uint8_t argc, i = 0;
    char *argv[30];
    char buf[50];
    cmd_t *cmd_entry;

    fflush(stdout);

    // parse the command line statement and break it up into space-delimited
    // strings. the array of strings will be saved in the argv array.
    argv[i] = strtok(cmd, " ");
    do
    {
      argv[++i] = strtok(NULL, " ");
    } while ((i < 30) && (argv[i] != NULL));
    // save off the number of arguments for the particular command.
    argc = i;
    // parse the command table for valid command. used argv[0] which is the
    // actual command name typed in at the prompt
    for (cmd_entry = _cmd_tbl; cmd_entry != NULL; cmd_entry = cmd_entry->next)
    {
      if (!strcmp(argv[0], cmd_entry->cmd))
      {
        cmd_entry->func(argc, argv);
        prompt();
        return;
      }
    }
    // command not recognized. print message and re-generate prompt.
    _serial->println(_err_msg);
    prompt();
  }

  void handler() {
    char c = _serial->read();

    switch (c)
    {
    // case '.':
    case '\r':
      // terminate the msg and reset the msg ptr. then send
      // it to the handler for processing.
      *_msg_ptr = '\0';
      _serial->print("\r\n");
      parse((char *)_msg);
      _msg_ptr = _msg;
      break;

    case '\n':
      // ignore newline characters. they usually come in pairs
      // with the \r characters we use for newline detection.
      break;

    case '\b':
      // backspace
      _serial->print(c);
      if (_msg_ptr > _msg)
      {
        _msg_ptr--;
      }
      break;

    default:
      // normal character entered. add it to the buffer
      if(c >= 32 && c <= 126) {
        _serial->print(c);
        *_msg_ptr++ = c;
      }
      break;
    }
  }
};

Cli cli;

#endif