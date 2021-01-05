/*****************************************************************************
 * SPI serial decoder for HP Agilent 16700 series logic analyzers.
 * Andrei A. Semenov <rtr989@gmail.com>
 *
 * See the LICENSE file accompying this source file for copyright and
 * redistribution information. Copyright (c) 2020 by Andrei A. Semenov.
 *
 * For use with the Tool Development Kit.
 * useSSline=1 for SS line
 *****************************************************************************/ 


enum conditions {
    IDLE,
    READ_DATA,
    INVALID
};

struct decoder {
    enum conditions     state;
    uint8_t             sck;
    uint8_t             sckPrev;
    uint8_t             mosi;
    uint8_t             miso;
    uint8_t             ss;
    uint8_t             pos;
    uint8_t             byte_mosi;
    uint8_t             byte_miso;
    long long           time;
};



enum { white, white2, scarlet, pumpkin, yellow, lime, turquoise, lavender };

typedef struct decoder decoder;

decoder* newDecoder();
void destroyDecoder(decoder *d);
void handleState(decoder *d, unsigned int sck, unsigned int ss, unsigned int mosi, unsigned int miso,
                 long long time, TDKDataSet &dataDs, TDKDataSet &eventDs,
                 TDKLabelEntry &spi_mosiLE, TDKLabelEntry &sPiEventsLE, TDKLabelEntry &spi_misoLE,
                 TDKBaseIO &io, int useSS);

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

    TDKLabelEntry spi_mosiLE;
    TDKLabelEntry sPiEventsLE;

    TDKLabelEntry spi_misoLE;

    long long correlationTime;
    unsigned int origNumSamples;
    int triggerRow;
    long long time, lastTime;

    int err = 0;

    int useSSline=0;

    
    int num = 0;
    num = sscanf( io.getArg( 0 ), "%i", &useSSline );
    if( num != 1 ){
        io.print( "Unable to convert use SS parameter" );
        return;
    }

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

    err = spi_mosiLE.createIntegralData(dataDs, "SPI_MOSI", 8);
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

    err = spi_misoLE.createIntegralData(dataDs, "SPI_MISO", 8);
    if(err) {
        io.printError(err);
        destroyDecoder(d);
        return;
    }


    while( ds.next(time) && sckLE.next(sckValue) && mosiLE.next(mosiValue) && misoLE.next(misoValue) && ssLE.next(ssValue) ) {
        handleState(d, sckValue, mosiValue, misoValue, ssValue, time, dataDs, eventDs, spi_mosiLE, sPiEventsLE, spi_misoLE, io, useSSline);
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
    d->sckPrev = 0;
    d->ss = 1;
    d->time = 0;

    return d;
}

void destroyDecoder(decoder *d) {
    free(d);
}

void handleState(decoder *d, unsigned int sck, unsigned int mosi, unsigned int miso, unsigned int ss,
                 long long time, TDKDataSet &dataDs, TDKDataSet &eventDs,
                 TDKLabelEntry &spi_mosiLE, TDKLabelEntry &sPiEventsLE, TDKLabelEntry &spi_misoLE,
                 TDKBaseIO &io, int useSS) {
                 
    if( d->sckPrev == sck && d->sck == sck ) {
        // nothing changed; move on to next state on the wire
        return;
    }

    // Look for stop condition. This can occur at any time.
    if(  d->sckPrev==1 && d->sck==0 && sck==0 && useSS==0 ) {
        dataDs.replaceNext(time);
        spi_mosiLE.setColor( spi_mosiLE.getPosition(), scarlet );
        spi_mosiLE.replaceNext((unsigned int)(0));
        spi_misoLE.setColor( spi_misoLE.getPosition(), scarlet );
        spi_misoLE.replaceNext((unsigned int)(0));

        eventDs.replaceNext(time);
        sPiEventsLE.setColor( sPiEventsLE.getPosition(), scarlet );
        sPiEventsLE.replaceNext((String)"STOP");

        d->state = IDLE;
    }
    if( d->ss==0 && ss==1  && useSS==1 ) {
        dataDs.replaceNext(time);
        spi_mosiLE.setColor( spi_mosiLE.getPosition(), scarlet );
        spi_mosiLE.replaceNext((unsigned int)(0));
        spi_misoLE.setColor( spi_misoLE.getPosition(), scarlet );
        spi_misoLE.replaceNext((unsigned int)(0));

        eventDs.replaceNext(time);
        sPiEventsLE.setColor( sPiEventsLE.getPosition(), scarlet );
        sPiEventsLE.replaceNext((String)"STOP");

        d->state = IDLE;
    }

    // Look for start condition. 
    if( d->sckPrev==0 && d->sck==0 && sck==1 && useSS==0 ) {
        if( d->state == IDLE ) {
            eventDs.replaceNext(time);
            sPiEventsLE.setColor( sPiEventsLE.getPosition(), lime );
            sPiEventsLE.replaceNext((String)"START");
        }
        d->state = READ_DATA;
        d->pos = 0;
        d->byte_mosi = 0;
        d->byte_miso = 0;
    }
    if( d->ss==1 && ss==0 && useSS==1 ) {
        if( d->state == IDLE ) {
            eventDs.replaceNext(time);
            sPiEventsLE.setColor( sPiEventsLE.getPosition(), lime );
            sPiEventsLE.replaceNext((String)"START");
        }
        d->state = READ_DATA;
        d->pos = 0;
        d->byte_mosi = 0;
        d->byte_miso = 0;
    }

    // Read a byte of data
    if( d->state==READ_DATA && sck==1 && d->sck !=1 ) {
        
        if( d->pos == 0 ) {
            d->time = time;
        }
        
        d->byte_mosi |= mosi << (7-d->pos);
        d->byte_miso |= miso << (7-d->pos);
        d->pos++;
        if( d->pos == 8 ) {
            dataDs.replaceNext(d->time);
            spi_mosiLE.setColor( spi_mosiLE.getPosition(), yellow );
            spi_mosiLE.replaceNext((unsigned int)(d->byte_mosi));
            spi_misoLE.setColor( spi_misoLE.getPosition(), lavender );
            spi_misoLE.replaceNext((unsigned int)(d->byte_miso));
            d->pos = 0;
            d->byte_mosi = 0;
            d->byte_miso = 0;
        }
    }



    d->ss = ss;
    d->sckPrev = d->sck;
    d->sck = sck;
}



StringList getLabelNames()
{
  StringList labels;
  labels.put("Use SS line: ");
  return labels;
}


// Assign default values to runtime arguments
StringList getDefaultArgs()
{
  StringList defs;
  defs.put("0");
  return defs;
}


