# oss-services/xmodem

## Overview

This component implements the x/ymodem send/receive protocol.

## Required components

- None

## Recommended components

- Shell

## Integrate the source

- Copy the 'src' directory contents into an appropriate source directory in the host project.
- Copy the 'inc' directory contents into an approporiate configuration include directory in the host project.

## Send / Receive example code

```C
typedef struct XMODEM_STATE {
    SHELL_CONTEXT *ctx;
} XMODEM_STATE;

static void shell_xmodem_putchar(int c, void *usr)
{
    XMODEM_STATE *x = (XMODEM_STATE *)usr;
    SHELL_CONTEXT *ctx = x->ctx;
    TERM_STATE *t = &ctx->t;

    term_putch(t, c);
}

static int shell_xmodem_getchar(int timeout, void *usr)
{
    XMODEM_STATE *x = (XMODEM_STATE *)usr;
    SHELL_CONTEXT *ctx = x->ctx;
    TERM_STATE *t = &ctx->t;
    int c;

    c = term_getch(t, timeout);

    return(c);
}

typedef struct FILE_XFER_STATE {
    XMODEM_STATE xmodem;
    FILE *f;
    void *data;
    int size;
} FILE_XFER_STATE;

void fileDataWrite(void *usr, void *data, int size)
{
    FILE_XFER_STATE *state = (FILE_XFER_STATE *)usr;
    size_t wsize;

    /*
     * Need to double-buffer the data in order to strip off the
     * trailing packet bytes at the end of an xmodem transfer.
     */
    if (state->data == NULL) {
        state->data = SHELL_MALLOC(1024);
        memcpy(state->data, data, size);
        state->size = size;
    } else {
        if (data) {
            wsize = fwrite(state->data, 1, state->size, state->f);
            memcpy(state->data, data, size);
            state->size = size;
        } else {
            uint8_t *buf = (uint8_t *)state->data;
            while (state->size && buf[state->size-1] == '\x1A') {
               state->size--;
            }
            wsize = fwrite(state->data, 1, state->size, state->f);
            if (state->data) {
                SHELL_FREE(state->data);
                state->data = NULL;
            }
        }
    }
}

void fileDataRead(void *usr, void *data, int size)
{
    FILE_XFER_STATE *state = (FILE_XFER_STATE *)usr;
    size_t rsize;

    if (size > 0) {
        rsize = fread(data, 1, size, state->f);
    }
}

void shell_recv( SHELL_CONTEXT *ctx, int argc, char **argv )
{
    FILE_XFER_STATE fileState = { 0 };
    long size;

    if( argc != 2 ) {
        printf( "Usage: recv <file>\n" );
        return;
    }

    fileState.xmodem.ctx = ctx;

    fileState.f = fopen( argv[ 1 ], "wb");
    if( fileState.f == NULL) {
        printf( "unable to open file %s\n", argv[ 1 ] );
        return;
    }
    printf( "Prepare your terminal for XMODEM send ... " );
    term_set_mode(&ctx->t, TERM_MODE_COOKED, 0);
    size = XmodemReceiveCrc(fileDataWrite, &fileState, INT_MAX,
        shell_xmodem_getchar, shell_xmodem_putchar);
    term_set_mode(&ctx->t, TERM_MODE_COOKED, 1);
    if (size < 0) {
        printf( "XMODEM Error: %ld\n", size);
    } else {
        printf( "received and saved as %s\n", argv[ 1 ] );
    }
    fclose( fileState.f );
}

typedef struct YMODEM_STATE {
    FILE_XFER_STATE state;
    const char *fname;
    size_t fsize;
} YMODEM_STATE;

static void ymodem_hdr(void *usr, void *xmodemBuffer, int xmodemSize)
{
    YMODEM_STATE *y = (YMODEM_STATE *)usr;
    snprintf(xmodemBuffer, xmodemSize, "%s%c%u", y->fname, 0, (unsigned)y->fsize);
}

static void ymodem_end(void *xs, void *xmodemBuffer, int xmodemSize)
{
}

void shell_send(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    size_t size;
    FILE *fp = NULL;
    int ret = -1;
    int i;

    YMODEM_STATE y = {
        .state.xmodem.ctx = ctx,
    };

    if (argc < 2) {
        printf("Usage: %s <file1> [<file2> ...]\n", argv[0]);
        return;
    }

    printf ("Prepare your terminal for YMODEM receive...\n");
    term_set_mode(&ctx->t, TERM_MODE_COOKED, 0);
    for (i = 1; i < argc; i++) {
        fp = fopen( argv[i], "rb");
        if (fp) {
            fseek(fp, 0, SEEK_END);
            size = ftell(fp);
            fseek(fp, 0, SEEK_SET);
            y.fname = argv[i]; y.fsize = size; y.state.f = fp;
            ret = XmodemTransmit(ymodem_hdr, &y, 128, 0, 1,
                shell_xmodem_getchar, shell_xmodem_putchar);
            if (ret >= 0) {
                ret = XmodemTransmit(fileDataRead, &y, y.fsize, 1, 0,
                    shell_xmodem_getchar, shell_xmodem_putchar);
            }
            fclose(fp);
            if (ret < 0) {
                break;
            }
        }
    }
    if (ret >= 0) {
        ret = XmodemTransmit(ymodem_end, &y, 128, 0, 1,
            shell_xmodem_getchar, shell_xmodem_putchar);
    }
    term_set_mode(&ctx->t, TERM_MODE_COOKED, 1);
    if (ret < 0) {
        printf( "YMODEM Error: %ld\n", ret);
    }
}
```
