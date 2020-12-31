

enum conditions {
    IDLE,
    READ_ADDR,
    READ_RW,
    READ_DATA,
    READ_ACK,
    INVALID
};

struct decoder {
    enum conditions     state;
    uint8_t             sck;
    uint8_t             mosi;
    uint8_t             miso;
    uint8_t             ss;
    uint8_t             pos;
    uint8_t             byte_mosi;
    uint8_t             byte_miso;
};

typedef struct decoder decoder;

decoder* newDecoder();
void destroyDecoder(decoder *d);
void handleState(decoder *d, unsigned int sck, unsigned int ss, unsigned int mosi, unsigned int miso,
                 long long time, TDKDataSet &dataDs, TDKDataSet &eventDs,
                 TDKLabelEntry &sPiLE, TDKLabelEntry &sPiEventsLE, TDKLabelEntry &sPiLE_miso,
                 TDKBaseIO &io);

void execute(TDKDataGroup &dg, TDKBaseIO &io) {
    decoder *d;

    TDKDataSet ds;
    TDKDataSet dataDs;
    TDKDataSet eventDs;

    TDKLabelEntry sckLE;
    TDKLabelEntry mosiLE;
    TDKLabelEntry misoLE;
    TDKLabelEntry ssLE;
    unsigned int sckValue, mosiValue, misoValue, ssValue;

    TDKLabelEntry sPiLE;
    TDKLabelEntry sPiEventsLE;

    TDKLabelEntry sPiLE_miso;

    long long correlationTime;
    unsigned int origNumSamples;
    int triggerRow;
    long long time, lastTime;

    int err = 0;

    d = newDecoder();
    if(d==0) {
        io.print("Unable to allocate decoder memory.");
        return;
    }

    err = ds.attach(dg);
    if(err) {
        io.printError(err);
        destroyDecoder(d);
        return;
    }

    correlationTime = ds.getCorrelationTime();
    ds.setTimeBias();

    err = sckLE.attach(ds, "SCK");
    if(err) {
        io.printError(err);
        destroyDecoder(d);
        return;
    }

    err = mosiLE.attach(ds, "MOSI");
    if(err) {
        io.printError(err);
        destroyDecoder(d);
        return;
    }

    err = misoLE.attach(ds, "MISO");
    if(err) {
        io.printError(err);
        destroyDecoder(d);
        return;
    }

    err = ssLE.attach(ds, "SS");
    if(err) {
        io.printError(err);
        destroyDecoder(d);
        return;
    }

    origNumSamples = ds.getNumberOfSamples();

    ds.peekNext(time);
    triggerRow = -1;

    while(time <= nanoSec(0)) {
        ds.next(time);
        triggerRow++;
    }
    triggerRow=0;

    ds.reset();

    err = dataDs.createTimeTags(dg, "SPIData", origNumSamples,
                                triggerRow, correlationTime, nanoSec(4.0));
    if(err) {
        io.printError(err);
        destroyDecoder(d);
        return;
    }
    dataDs.setTimeBias();
    dataDs.reset();
    dataDs.displayStateNumberLabel(false);

    err = eventDs.createTimeTags(dg, "SPIEvents", origNumSamples,
                                 triggerRow, correlationTime, nanoSec(4.0));
    if(err) {
        io.printError(err);
        destroyDecoder(d);
        return;
    }
    eventDs.setTimeBias();
    eventDs.reset();
    eventDs.displayStateNumberLabel(false);

    err = sPiLE.createIntegralData(dataDs, "SPI_MOSI", 8);
    if(err) {
        io.printError(err);
        destroyDecoder(d);
        return;
    }

    err = sPiEventsLE.createTextData(eventDs, "SPI_EVENT", 16);
    if(err) {
        io.printError(err);
        destroyDecoder(d);
        return;
    }

    err = sPiLE_miso.createIntegralData(dataDs, "SPI_MISO", 8);
    if(err) {
        io.printError(err);
        destroyDecoder(d);
        return;
    }


    while( ds.next(time) && sckLE.next(sckValue) && mosiLE.next(mosiValue) && misoLE.next(misoValue) && ssLE.next(ssValue) ) {
        handleState(d, sckValue, mosiValue, misoValue, ssValue, time, dataDs, eventDs, sPiLE, sPiEventsLE, sPiLE_miso, io);
        lastTime = time;
    }

    // try to clean up rest of new data sets. Make sure time always goes
    // forward, and filter data points we didn't end up using.
    long long eventPosition;
    long long dataPosition;

    eventDs.setStateBias();
    dataDs.setStateBias();

    eventPosition = eventDs.getPosition();
    dataPosition = dataDs.getPosition();

    eventDs.setTimeBias();
    dataDs.setTimeBias();

    while(eventDs.replaceNext(++lastTime));
    while(dataDs.replaceNext(++lastTime));

    eventDs.filterAllStates();
    dataDs.filterAllStates();

    long long f;
    for(f=0; f<eventPosition; f++) {
        eventDs.unfilter(f);
    }

    for(f=0; f<dataPosition; f++) {
        dataDs.unfilter(f);
    }

    destroyDecoder(d);

    dg.setTimeCrossCorrelation();

}

decoder* newDecoder() {
    decoder *d;
    d = (decoder*)malloc(sizeof(decoder));

    // Assume we're starting with an idle bus
    d->state = IDLE;
    d->sck = 0;
    d->mosi = 1;
    d->miso = 1;
    d->ss = 1;

    return d;
}

void destroyDecoder(decoder *d) {
    free(d);
}

void handleState(decoder *d, unsigned int sck, unsigned int mosi, unsigned int miso, unsigned int ss,
                 long long time, TDKDataSet &dataDs, TDKDataSet &eventDs,
                 TDKLabelEntry &sPiLE, TDKLabelEntry &sPiEventsLE, TDKLabelEntry &sPiLE_miso,
                 TDKBaseIO &io) {
    if( d->ss == ss && ss==1 ) {
        // nothing changed; move on to next state on the wire
        return;
    }


    // Look for start condition. This can occur at any time, even if bus isn't
    // idle.
    else if( d->ss==1 && ss==0) {
        if( d->state == IDLE ) {
            eventDs.replaceNext(time);
            sPiEventsLE.replaceNext((String)"START");
        }
        d->state = READ_DATA;
        d->pos = 0;
        d->byte_mosi = 0;
        d->byte_miso = 0;
    }

    // Look for stop condition. This can occur at any time.
    else if( d->ss==0 && ss==1 ) {
        eventDs.replaceNext(time);
        sPiEventsLE.replaceNext((String)"STOP");
        d->state = IDLE;
    }



    // Read a byte of data
    else if( d->state==READ_DATA && sck==1 ) {
        d->byte_mosi |= mosi << (7-d->pos);
        d->byte_miso |= miso << (7-d->pos);
        d->pos++;
        if( d->pos == 8 ) {
            dataDs.replaceNext(time);
            sPiLE.replaceNext((unsigned int)(d->byte_mosi));
            sPiLE_miso.replaceNext((unsigned int)(d->byte_miso));
            eventDs.replaceNext(time);
            sPiEventsLE.replaceNext((String)"DATA");
            d->pos = 0;
            d->byte_mosi = 0;
            d->byte_miso = 0;
            d->state = IDLE;
        }
    }



    d->ss = ss;
    d->sck = sck;
    d->mosi = mosi;
    d->miso = miso;
}

StringList getLabelNames() {
    StringList labels;
    return labels;
}

StringList getDefaultArgs() {
    StringList labels;
    return labels;
}


